// Loads recorded runs and replays them. Time here is the simulation's tick count, advanced by
// the browser's clock only to decide which recorded tick to show.

import { ArmView } from "./arm-view.js";
import { drawAcross, drawSeries, formatDistance } from "./charts.js";

const TRACE_DIRECTORY = "traces";

const elements = Object.fromEntries(
  [
    "scenario", "arm-view", "across-view", "series-view", "banner", "title", "description",
    "play", "scrub", "speed", "r-time", "r-depth", "r-radial", "r-monitor",
  ].map((id) => [id, document.getElementById(id)]),
);

const armView = new ArmView(elements["arm-view"]);
const state = { trace: null, tick: 0, playing: false, lastFrameMs: 0 };

async function fetchJson(path) {
  const response = await fetch(path);
  if (!response.ok) {
    throw new Error(`${path}: ${response.status}`);
  }
  return response.json();
}

// The arm pose at `tick`, interpolated between the two recorded frames around it.
function chainAt(trace, tick) {
  const { every_ticks: every, chain_m: frames } = trace.frames;
  const position = Math.min(tick / every, frames.length - 1);
  const before = Math.floor(position);
  const after = Math.min(before + 1, frames.length - 1);
  const blend = position - before;
  return frames[before].map((point, i) =>
    point.map((value, axis) => value + blend * (frames[after][i][axis] - value)),
  );
}

function render() {
  const { trace, tick } = state;
  const { events, series } = trace;
  const stopped = events.stop_tick !== null && tick >= events.stop_tick;
  const faulted = events.fault_start_tick !== null && tick >= events.fault_start_tick;

  armView.setPose(chainAt(trace, tick), stopped);
  drawAcross(elements["across-view"], trace, tick);
  drawSeries(elements["series-view"], trace, tick);

  elements["r-time"].textContent = `${(tick * trace.period_s).toFixed(3)} s`;
  elements["r-depth"].textContent = `${series.depth_mm[tick].toFixed(2)} mm`;
  elements["r-radial"].textContent = formatDistance(series.radial_um[tick]);
  elements["r-monitor"].textContent = stopped
    ? "stopped"
    : trace.monitored
      ? "watching"
      : "disabled";
  elements["r-monitor"].classList.toggle("stopped", stopped);

  const banner = elements.banner;
  banner.hidden = !faulted && !stopped;
  banner.classList.toggle("fault", faulted && !stopped);
  if (stopped) {
    const cause = events.stop_cause.replaceAll("_", " ");
    const delay = events.detection_ms === null ? "" : `, ${events.detection_ms} ms after the fault`;
    banner.textContent = `Safety stop: ${cause}${delay}`;
  } else if (faulted) {
    banner.textContent = "Fault injected";
  }
  elements.scrub.value = String(tick);
}

function setTick(tick) {
  state.tick = Math.max(0, Math.min(Math.round(tick), state.trace.series.radial_um.length - 1));
  render();
}

function setPlaying(playing) {
  state.playing = playing;
  state.lastFrameMs = 0;
  elements.play.textContent = playing ? "Pause" : "Play";
  if (playing) {
    requestAnimationFrame(advance);
  }
}

function advance(nowMs) {
  if (!state.playing) {
    return;
  }
  if (state.lastFrameMs !== 0) {
    const elapsedS = (nowMs - state.lastFrameMs) / 1000;
    const ticks = (elapsedS * Number(elements.speed.value)) / state.trace.period_s;
    const last = state.trace.series.radial_um.length - 1;
    if (state.tick + ticks >= last) {
      setTick(last);
      setPlaying(false);
      return;
    }
    // Kept as a fraction so that slow playback still advances.
    state.exact = (state.exact ?? state.tick) + ticks;
    setTick(state.exact);
  }
  state.lastFrameMs = nowMs;
  requestAnimationFrame(advance);
}

async function loadScenario(file) {
  setPlaying(false);
  const trace = await fetchJson(`${TRACE_DIRECTORY}/${file}`);
  state.trace = trace;
  state.exact = 0;
  elements.title.textContent = trace.title;
  elements.description.textContent = trace.description;
  elements.scrub.max = String(trace.series.radial_um.length - 1);
  armView.setTunnel(trace.tunnel.entry_m, trace.tunnel.target_m);
  setTick(0);
}

async function start() {
  let index;
  try {
    index = await fetchJson(`${TRACE_DIRECTORY}/index.json`);
  } catch (error) {
    elements.title.textContent = "No recorded runs found";
    elements.description.textContent =
      "Generate them with `python -m stapes export viewer/traces`, then serve this directory " +
      `over HTTP. (${error.message})`;
    return;
  }
  for (const scenario of index.scenarios) {
    const option = document.createElement("option");
    option.value = scenario.file;
    option.textContent = scenario.title;
    elements.scenario.append(option);
  }
  // A link can name a scenario: index.html#frozen-motor-encoder
  const wanted = `${window.location.hash.slice(1)}.json`;
  const initial = index.scenarios.find((scenario) => scenario.file === wanted) ?? index.scenarios[0];
  elements.scenario.value = initial.file;
  await loadScenario(initial.file);

  elements.scenario.addEventListener("change", () => {
    window.location.hash = elements.scenario.value.replace(".json", "");
    loadScenario(elements.scenario.value);
  });
  elements.play.addEventListener("click", () => {
    const last = state.trace.series.radial_um.length - 1;
    if (!state.playing && state.tick >= last) {
      state.exact = 0;
      setTick(0);
    }
    setPlaying(!state.playing);
  });
  elements.scrub.addEventListener("input", () => {
    setPlaying(false);
    state.exact = Number(elements.scrub.value);
    setTick(state.exact);
  });
  document.addEventListener("keydown", (event) => {
    if (event.target instanceof HTMLSelectElement || event.target instanceof HTMLInputElement) {
      return;
    }
    if (event.code === "Space") {
      event.preventDefault();
      elements.play.click();
    } else if (event.code === "ArrowRight" || event.code === "ArrowLeft") {
      setPlaying(false);
      state.exact = state.tick + (event.code === "ArrowRight" ? 10 : -10);
      setTick(state.exact);
    }
  });
}

start();
