// Hands-on mode. Reads a gamepad (or the keyboard), sends what the operator is doing to the
// local simulation once per frame, and draws what comes back. Nothing here decides anything:
// speed limits, alignment and stops are all the simulation's and the monitor's.

import * as THREE from "three";
import { ArmView } from "./arm-view.js";
import { drawAcross, formatDistance } from "./charts.js";

const DEADZONE = 0.15;
// The browser can drop frames; the simulation never advances more than this per request.
const MAX_TICKS = 50;
// How many frames the trail in the across view keeps.
const TRAIL_FRAMES = 240;

// The standard gamepad mapping, which Xbox controllers follow in every major browser.
const PAD = { a: 0, b: 1, x: 2, y: 3, lb: 4, rb: 5, view: 8, start: 9 };
const JOINT_COUNT = 6;
// The standard mapping's sticks: left is axes 0 and 1, right is axes 2 and 3.
const AXIS = { leftX: 0, leftY: 1, rightX: 2, rightY: 3 };

const elements = Object.fromEntries(
  [
    "arm-view", "across-view", "banner", "mode", "hint", "pad", "fault", "reset", "follow",
    "r-time", "r-depth", "r-radial", "r-monitor",
  ].map((id) => [id, document.getElementById(id)]),
);

const armView = new ArmView(elements["arm-view"]);
const keys = new Set();
const state = { scene: null, live: null, inFlight: false, lastSentMs: 0, lastFrameMs: 0, mode: "free" };
// How fast the view turns at full deflection, in rad/s. Much faster than the arm: it is only the
// camera.
const VIEW_TURN_RAD_S = { azimuth: 2.4, elevation: 1.6 };
// Presses seen since the last request. A press shorter than a request's round trip still counts.
const latched = { align: false, fault: false, jointMode: false };
// The joint the operator is turning in joint mode, from 0 at the base. The page keeps the
// choice and sends it with every input, so a recording holds it too.
let chosenJoint = 0;

const HINTS = {
  free:
    "Left stick moves the tip across, right stick up and down. A: the arm aligns itself with " +
    "the plan. View: turn one joint at a time.",
  joint:
    "LB and RB choose the joint, the left stick turns it. Each joint stops softly before its " +
    "limit. View: back to moving the tip.",
  aligning: "Moving onto the planned line, in joint space.",
  on_line:
    "On the line: right stick up drills, down backs out. The left stick does nothing here. " +
    "Back at the standoff, A lets go.",
};
const REFUSALS = {
  still_moving: "Let the tip stop first: the move onto the line starts from rest.",
  enters_guard: "Refused: that move would cut inside the entry guard. Back away from the bone first.",
  not_at_standoff: "Back out to the standoff before letting go of the line.",
};
const MODE_TITLES = { free: "Free", aligning: "Aligning", on_line: "On the line", joint: "Joint" };

function deadzone(value) {
  return Math.abs(value) < DEADZONE ? 0 : (value - Math.sign(value) * DEADZONE) / (1 - DEADZONE);
}

// The controller as the server reads it from Linux, when it does: more reliable than the
// browser's guess at a controller it does not know. Updated with every reply.
let serverPad = null;

function gamepad() {
  if (serverPad !== null) {
    return serverPad;
  }
  return [...(navigator.getGamepads?.() ?? [])].find((pad) => pad && pad.connected) ?? null;
}

async function api(path, body) {
  const reply = await request(path, body);
  if ("pad" in reply) {
    serverPad = reply.pad;
  }
  return reply;
}

// The operator's input now, from the gamepad if there is one, plus the keyboard.
function readInput() {
  const pad = gamepad();
  const pressed = (index) => Boolean(pad?.buttons[index]?.pressed);
  const key = (code) => (keys.has(code) ? 1 : 0);

  const axis = (index) => deadzone(pad?.axes[index] ?? 0);
  const across = axis(AXIS.leftX) + key("KeyD") - key("KeyA");
  const forward = -axis(AXIS.leftY) + key("KeyW") - key("KeyS");
  // Right stick up: the tip goes up, or on the line, into the bone.
  const lift = -axis(AXIS.rightY) + key("KeyE") - key("KeyQ");
  const lookX = axis(AXIS.rightX);
  const lookY = axis(AXIS.rightY);
  return {
    across: Math.max(-1, Math.min(1, across)),
    forward: Math.max(-1, Math.min(1, forward)),
    lift: Math.max(-1, Math.min(1, lift)),
    lookX,
    lookY,
    keyLookX: key("ArrowRight") - key("ArrowLeft"),
    keyLookY: key("ArrowDown") - key("ArrowUp"),
    align: pressed(PAD.a) || keys.has("Space"),
    fault: pressed(PAD.b) || keys.has("KeyX"),
    // X and Y swap places on some controllers; neither has another use here.
    follow: pressed(PAD.y) || pressed(PAD.x) || keys.has("KeyF"),
    jointMode: pressed(PAD.view) || keys.has("KeyJ"),
    previousJoint: pressed(PAD.lb),
    nextJoint: pressed(PAD.rb),
    reset: pressed(PAD.start) || keys.has("Backspace"),
    resetSource: pressed(PAD.start) ? "gamepad Start" : "Backspace",
  };
}

// The stick moves the tip relative to the camera, in the horizontal plane: up on the stick is
// away from the viewer. This is input mapping only; the simulation receives a world velocity.
function tipVelocity(input) {
  const speed = state.scene.max_tip_speed_m_s;
  const forward = new THREE.Vector3();
  armView.camera.getWorldDirection(forward);
  forward.z = 0;
  if (forward.lengthSq() < 1e-9) {
    forward.set(1, 0, 0);
  }
  forward.normalize();
  const right = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 0, 1));
  const velocity = forward
    .multiplyScalar(input.forward)
    .addScaledVector(right, input.across)
    .add(new THREE.Vector3(0, 0, input.lift))
    .multiplyScalar(speed);
  return [velocity.x, velocity.y, velocity.z];
}

async function request(path, body) {
  const response = await fetch(path, body === undefined ? {} : {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!response.ok) {
    throw new Error(`${path}: ${response.status}`);
  }
  return response.json();
}

// The across view draws a recorded trace. Here the "trace" is the recent frames, rebuilt as
// they arrive, in the same layout.
function newLive(scene) {
  return {
    tunnel: scene.tunnel,
    limits: scene.limits,
    events: { stop_tick: null },
    series: { depth_mm: [], radial_um: [], across_u_um: [], across_v_um: [] },
  };
}

function pushLive(live, sample) {
  const { series } = live;
  for (const key of Object.keys(series)) {
    series[key].push(sample[key]);
    if (series[key].length > TRAIL_FRAMES) {
      series[key].shift();
    }
  }
  const last = series.depth_mm.length - 1;
  live.events.stop_tick = sample.stop_tick === null ? null : last;
  return last;
}

function render(sample) {
  const stopped = sample.stop_tick !== null;
  const index = pushLive(state.live, sample);
  armView.setPose(sample.chain_m, stopped);
  drawAcross(elements["across-view"], state.live, index);

  setText(elements["r-time"], `${(sample.tick * state.scene.period_s).toFixed(2)} s`);
  setText(elements["r-depth"], `${sample.depth_mm.toFixed(2)} mm`);
  setText(
    elements["r-radial"],
    sample.depth_mm > -state.scene.tunnel.entry_guard_mm
      ? formatDistance(sample.radial_um)
      : `${(sample.radial_um / 1000).toFixed(1)} mm`,
  );
  setText(elements["r-monitor"], stopped ? "stopped" : "watching");
  elements["r-monitor"].classList.toggle("stopped", stopped);

  state.mode = sample.mode;
  const inJointMode = sample.mode === "joint";
  armView.setChosenJoint(inJointMode && !stopped ? chosenJoint : null);
  setText(
    elements.mode,
    stopped
      ? "Stopped"
      : inJointMode
        ? `Joint ${chosenJoint + 1} of ${JOINT_COUNT}`
        : MODE_TITLES[sample.mode],
  );
  setText(
    elements.hint,
    stopped
      ? "The monitor has stopped the robot and holds the brakes. Press Start (Backspace) to begin again."
      : (REFUSALS[sample.refusal] ?? HINTS[sample.mode]),
  );

  const faulted = sample.fault_start_tick !== null;
  const banner = elements.banner;
  banner.hidden = !faulted && !stopped;
  banner.classList.toggle("fault", faulted && !stopped);
  if (stopped) {
    const cause = sample.stop_cause.replaceAll("_", " ");
    const delay = faulted ? `, ${sample.stop_tick - sample.fault_start_tick} ms after the fault` : "";
    setText(banner, `Safety stop: ${cause}${delay}`);
  } else if (faulted) {
    setText(banner, "Fault injected");
  }
}

async function reset(reason) {
  const opened = await api("/api/jog/reset", { fault: elements.fault.value, reason });
  start(opened);
}

function start({ scene, state: sample }) {
  state.scene = scene;
  state.lastSentMs = 0;
  state.live = newLive(scene);
  armView.setTunnel(scene.tunnel.entry_m, scene.tunnel.target_m);
  render(sample);
}

// The right stick turns the view in joint mode, where it has nothing else to do; the arrow keys
// turn it in any mode. Squared, so small movements are fine and full deflection is fast.
function viewTurn(input) {
  const shape = (value) => value * Math.abs(value);
  const stickX = state.mode === "joint" ? input.lookX : 0;
  const stickY = state.mode === "joint" ? input.lookY : 0;
  const x = Math.max(-1, Math.min(1, shape(stickX) + input.keyLookX));
  const y = Math.max(-1, Math.min(1, shape(stickY) + input.keyLookY));
  return { azimuth: -x * VIEW_TURN_RAD_S.azimuth, elevation: -y * VIEW_TURN_RAD_S.elevation };
}

// Writes text only when it changes, so the page does not lay itself out again for nothing.
function setText(element, text) {
  if (element.textContent !== text) {
    element.textContent = text;
  }
}

function chooseJoint(step) {
  chosenJoint = (chosenJoint + step + JOINT_COUNT) % JOINT_COUNT;
}

let previous = {};
async function frame(nowMs) {
  requestAnimationFrame(frame);
  if (state.scene === null) {
    return;
  }
  const input = readInput();
  latched.align ||= input.align;
  latched.fault ||= input.fault;
  latched.jointMode ||= input.jointMode;
  const pad = gamepad();
  if (pad) {
    setText(elements.pad, `Gamepad: ${pad.id}`);
  }

  // The view turns every frame, whether or not a request is on its way: it is the page's own.
  const elapsedS = state.lastFrameMs === 0 ? 0 : Math.min((nowMs - state.lastFrameMs) * 1e-3, 0.1);
  state.lastFrameMs = nowMs;
  const look = viewTurn(input);
  if (look.azimuth !== 0 || look.elevation !== 0) {
    armView.orbit(look.azimuth * elapsedS, look.elevation * elapsedS);
  }

  // Buttons the page handles itself act on the press, not while held.
  const followPressed = input.follow && !previous.follow;
  if (input.nextJoint && !previous.nextJoint) {
    chooseJoint(1);
  }
  if (input.previousJoint && !previous.previousJoint) {
    chooseJoint(-1);
  }
  const resetPressed = input.reset && !previous.reset;
  previous = input;
  if (followPressed) {
    elements.follow.click();
  }
  if (state.inFlight) {
    return;
  }
  if (resetPressed) {
    state.inFlight = true;
    await reset(input.resetSource).finally(() => {
      state.inFlight = false;
    });
    return;
  }

  // Simulated time follows the browser's clock from one request to the next.
  if (state.lastSentMs === 0) {
    state.lastSentMs = nowMs;
    return;
  }
  const ticks = Math.min(
    Math.round(((nowMs - state.lastSentMs) * 1e-3) / state.scene.period_s),
    MAX_TICKS,
  );
  if (ticks === 0) {
    return;
  }
  state.lastSentMs = nowMs;
  // In joint mode the right stick turns the view, so it commands nothing.
  const lift = state.mode === "joint" ? 0 : input.lift;
  const free = state.mode === "joint" ? { ...input, lift: 0 } : input;
  const values = [
    ...tipVelocity(free), lift, latched.align, latched.fault,
    latched.jointMode, chosenJoint, input.across,
  ];
  latched.align = input.align;
  latched.fault = input.fault;
  latched.jointMode = input.jointMode;
  state.inFlight = true;
  try {
    render(await api("/api/jog/step", { ticks, input: values }));
  } catch (error) {
    elements.mode.textContent = "Lost the simulation";
    elements.hint.textContent = `Is python -m stapes jog still running? (${error.message})`;
  } finally {
    state.inFlight = false;
  }
}

async function main() {
  try {
    start(await api("/api/jog/scene"));
  } catch (error) {
    elements.mode.textContent = "No live simulation here";
    elements.hint.textContent =
      "The hands-on mode needs the simulation running on this machine: " +
      `python -m stapes jog, then open the address it prints. (${error.message})`;
    return;
  }
  document.addEventListener("keydown", (event) => {
    if (event.target instanceof HTMLSelectElement) {
      return;
    }
    keys.add(event.code);
    // A tap can come and go between two frames; the press is kept until it is sent.
    latched.align ||= event.code === "Space";
    latched.fault ||= event.code === "KeyX";
    latched.jointMode ||= event.code === "KeyJ";
    if (!event.repeat && (event.code === "BracketLeft" || event.code === "BracketRight")) {
      chooseJoint(event.code === "BracketRight" ? 1 : -1);
    }
    // Some controllers also send a Home key, which would take the browser off this page.
    const handled = ["Space", "Backspace", "BrowserHome", "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"];
    if (handled.includes(event.code) || event.key === "BrowserHome") {
      event.preventDefault();
    }
  });
  document.addEventListener("keyup", (event) => keys.delete(event.code));
  window.addEventListener("blur", () => keys.clear());
  window.addEventListener("gamepadconnected", (event) => {
    elements.pad.textContent = `Gamepad: ${event.gamepad.id}`;
  });
  // The page's own buttons are for the mouse only. Kept from the focus, they cannot be pressed
  // by a key, or by a controller that also sends keys.
  for (const control of [elements.reset, elements.follow]) {
    control.addEventListener("mousedown", (event) => event.preventDefault());
    control.addEventListener("keydown", (event) => event.preventDefault());
    control.addEventListener("keyup", (event) => event.preventDefault());
  }
  elements.reset.addEventListener("click", (event) => {
    if (event.detail > 0) {
      reset("Start again button");
    }
  });
  elements.fault.addEventListener("change", () => {
    elements.fault.blur();
    reset("fault changed");
  });
  elements.follow.addEventListener("click", () => {
    const following = elements.follow.getAttribute("aria-pressed") !== "true";
    armView.setFollowing(following);
    elements.follow.setAttribute("aria-pressed", String(following));
    elements.follow.textContent = following ? "Wide view" : "Follow tip";
  });
  // Start with the chosen fault armed.
  await reset("page opened");
  requestAnimationFrame(frame);
}

main();
