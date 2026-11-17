"""Hands-on mode: drive the simulated arm from a gamepad in the browser, live.

The page reads the gamepad and sends what the operator is doing, once per animation frame. This
module steps the simulation by as many control periods as the browser clock has moved, and
returns what the viewer draws. It computes nothing the core does not: positions come from the
core's kinematics, and every decision is the simulation's or the monitor's.

Each session is a sequence of inputs, so it is recorded as one and replays exactly: the same
seed and the same inputs give the same state hash.
"""

import functools
import json
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from stapes_core import (
    Fault,
    FaultKind,
    JogSimulation,
    KeyholeScenario,
    OperatorInput,
    StopCause,
    align_refusal_name,
    chain_positions,
    jog_mode_name,
    stop_cause_name,
    with_approach,
)

from .gamepad import LinuxGamepad
from .viewer_export import _perpendicular_basis

RECORDING_FORMAT = 2
# Faults the operator can start. Each one affects joint 2 where it affects one joint, as the
# recorded scenarios do.
FAULTS = {
    "none": FaultKind.none,
    "frozen_motor_encoder": FaultKind.frozen_motor_encoder,
    "frozen_link_encoder": FaultKind.frozen_link_encoder,
    "frozen_tracker": FaultKind.frozen_tracker,
    "stuck_actuator": FaultKind.stuck_actuator,
    "comms_delay": FaultKind.comms_delay,
}
FAULT_JOINT = 1
COMMS_DELAY_TICKS = 20
# The most control periods one request may advance. A browser tab in the background stops
# sending frames; when it comes back, the arm should not jump ahead by the time it was away.
MAX_TICKS_PER_REQUEST = 50


def jog_scenario(seed: int, fault: str) -> KeyholeScenario:
    scenario = with_approach(KeyholeScenario.reference(seed))
    scenario.fault = Fault(kind=FAULTS[fault], joint=FAULT_JOINT,
                           delay_ticks=COMMS_DELAY_TICKS if fault == "comms_delay" else 0)
    return scenario


# One input, as the page sends it and a recording stores it.
INPUT_FIELDS = ("vx", "vy", "vz", "feed", "align", "fault", "joint_mode", "joint", "joint_rate")


def normalised(values: list) -> list:
    """The input with each field in its own type, so that equal inputs compare equal."""
    if len(values) != len(INPUT_FIELDS):
        raise ValueError(f"an input has {len(INPUT_FIELDS)} values: {', '.join(INPUT_FIELDS)}")
    vx, vy, vz, feed, align, fault, joint_mode, joint, joint_rate = values
    return [float(vx), float(vy), float(vz), float(feed), bool(align), bool(fault),
            bool(joint_mode), int(joint), float(joint_rate)]


def operator_input(values: list) -> OperatorInput:
    """An input from its recorded form."""
    vx, vy, vz, feed, align, fault, joint_mode, joint, joint_rate = normalised(values)
    return OperatorInput(tip_velocity_m_s=np.array([vx, vy, vz]), feed=feed, align=align,
                         fault=fault, joint_mode=joint_mode, joint=joint, joint_rate=joint_rate)


@dataclass
class JogSession:
    """One run of the hands-on mode, and the inputs that drove it."""

    seed: int = 1
    fault: str = "none"
    # Runs of equal input: [first tick, count, *INPUT_FIELDS].
    inputs: list[list] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.fault not in FAULTS:
            raise ValueError(f"unknown fault: {self.fault}")
        self.scenario = jog_scenario(self.seed, self.fault)
        self.sim = JogSimulation(self.scenario)
        tunnel = self.scenario.trajectory
        self._u, self._v = _perpendicular_basis(tunnel.direction)

    def advance(self, ticks: int, values: list) -> None:
        """Steps `ticks` control periods with the same input, and records it."""
        ticks = max(0, min(int(ticks), MAX_TICKS_PER_REQUEST))
        if ticks == 0:
            return
        values = normalised(values)
        step_input = operator_input(values)
        if self.inputs and self.inputs[-1][2:] == values:
            self.inputs[-1][1] += ticks
        else:
            self.inputs.append([self.sim.ticks, ticks, *values])
        for _ in range(ticks):
            self.sim.step(step_input)

    def scene(self) -> dict:
        """What does not change during a session: the plan and the limits."""
        tunnel = self.scenario.trajectory
        limits = self.scenario.safety
        return {
            "period_s": limits.period_s,
            "fault": self.fault,
            "max_tip_speed_m_s": JogSimulation.MAX_TIP_SPEED_M_S,
            "tunnel": {
                "entry_m": tunnel.entry.tolist(),
                "target_m": tunnel.target.tolist(),
                "length_mm": tunnel.length_m * 1e3,
                "entry_guard_mm": limits.entry_guard_m * 1e3,
                "standoff_mm": self.scenario.approach.standoff_m * 1e3,
            },
            "limits": {
                "design_target_um": 200.0,
                "corridor_radius_um": limits.corridor_radius_m * 1e6,
            },
        }

    def state(self) -> dict:
        """What the viewer draws now."""
        sim = self.sim
        tunnel = self.scenario.trajectory
        tip = sim.true_tool_pose()[:3, 3]
        from_entry = tip - tunnel.entry
        stopped = sim.stop_tick is not None
        return {
            "tick": sim.ticks,
            "mode": jog_mode_name(sim.mode),
            "refusal": align_refusal_name(sim.align_refusal),
            "chain_m": np.round(chain_positions(self.scenario.arm, sim.joint_positions), 6).tolist(),
            "depth_mm": round(float(tunnel.depth_of(tip)) * 1e3, 4),
            "planned_depth_mm": round(sim.planned_depth_m * 1e3, 4),
            "radial_um": round(float(tunnel.radial_distance(tip)) * 1e6, 1),
            "across_u_um": round(float(from_entry @ self._u) * 1e6, 1),
            "across_v_um": round(float(from_entry @ self._v) * 1e6, 1),
            "fault_start_tick": sim.fault_start_tick,
            "stop_tick": sim.stop_tick,
            "stop_cause": stop_cause_name(sim.cause) if stopped else None,
        }

    def recording(self) -> dict:
        return {
            "format": RECORDING_FORMAT,
            "seed": self.seed,
            "fault": self.fault,
            "ticks": self.sim.ticks,
            "state_hash": f"{self.sim.state_hash():016x}",
            "stop_cause": stop_cause_name(self.sim.cause)
            if self.sim.cause != StopCause.none else None,
            "inputs": self.inputs,
        }


def replay(recording: dict) -> JogSession:
    """Runs a recorded session again from its inputs alone."""
    if recording.get("format") != RECORDING_FORMAT:
        raise ValueError(f"recording format {recording.get('format')}; expected {RECORDING_FORMAT}")
    session = JogSession(seed=recording["seed"], fault=recording["fault"])
    for first_tick, count, *values in recording["inputs"]:
        if session.sim.ticks != first_tick:
            raise ValueError(f"inputs skip from tick {session.sim.ticks} to {first_tick}")
        step_input = operator_input(values)
        for _ in range(count):
            session.sim.step(step_input)
    session.inputs = recording["inputs"]
    return session


class _Server(ThreadingHTTPServer):
    def __init__(self, address, viewer: Path, recordings: Path, seed: int,
                 gamepad: LinuxGamepad | None = None):
        self.gamepad = gamepad
        self.viewer = viewer
        self.recordings = recordings
        self.seed = seed
        self.lock = threading.Lock()
        self.session = JogSession(seed=seed)
        super().__init__(address, functools.partial(_Handler, directory=str(viewer)))

    def save(self) -> Path | None:
        """Writes the current session, if anything happened in it."""
        if self.session.sim.ticks == 0:
            return None
        self.recordings.mkdir(parents=True, exist_ok=True)
        path = self.recordings / f"jog-{time.strftime('%Y%m%d-%H%M%S')}-{self.session.fault}.json"
        path.write_text(json.dumps(self.session.recording(), separators=(",", ":")) + "\n",
                        encoding="utf-8")
        return path


class _Handler(SimpleHTTPRequestHandler):
    server: _Server

    def log_message(self, format, *args) -> None:  # noqa: A002 - the base class's name
        if not self.path.startswith("/api/"):
            super().log_message(format, *args)

    def _with_pad(self, body: dict) -> dict:
        """Adds the controller's state, when the server reads it, for the page to use."""
        gamepad = self.server.gamepad
        body["pad"] = gamepad.snapshot() if gamepad is not None else None
        return body

    def _reply(self, body: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(body, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        return json.loads(self.rfile.read(length) or b"{}")

    def do_GET(self) -> None:
        if self.path == "/api/jog/scene":
            with self.server.lock:
                self._reply(self._with_pad({"scene": self.server.session.scene(),
                                            "state": self.server.session.state()}))
            return
        super().do_GET()

    def do_POST(self) -> None:
        try:
            body = self._body()
            with self.server.lock:
                if self.path == "/api/jog/step":
                    self.server.session.advance(body["ticks"], body["input"])
                    self._reply(self._with_pad(self.server.session.state()))
                elif self.path == "/api/jog/reset":
                    saved = self.server.save()
                    print(f"reset: {body.get('reason', 'no reason given')}")
                    if saved is not None:
                        print(f"recorded {saved}")
                    self.server.session = JogSession(seed=self.server.seed,
                                                     fault=body.get("fault", "none"))
                    self._reply(self._with_pad({"scene": self.server.session.scene(),
                                                "state": self.server.session.state()}))
                else:
                    self._reply({"error": "not found"}, HTTPStatus.NOT_FOUND)
        except (KeyError, TypeError, ValueError) as error:
            self._reply({"error": str(error)}, HTTPStatus.BAD_REQUEST)


def serve(viewer: Path, recordings: Path, port: int = 8765, seed: int = 1,
          read_gamepad: bool = True) -> None:
    """Serves the viewer and the live simulation on localhost until interrupted."""
    gamepad = LinuxGamepad.open_first() if read_gamepad else None
    server = _Server(("127.0.0.1", port), viewer.resolve(), recordings, seed, gamepad)
    if gamepad is not None:
        print(f"Gamepad: {gamepad.snapshot()['id']}, {gamepad.path}")
    else:
        print("Gamepad: none read from Linux; the page uses the browser's")
    print(f"Hands-on mode: http://127.0.0.1:{port}/jog.html  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with server.lock:
            saved = server.save()
        if saved is not None:
            print(f"recorded {saved}")
        server.server_close()
        if gamepad is not None:
            gamepad.close()
