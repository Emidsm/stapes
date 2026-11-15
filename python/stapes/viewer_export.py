"""Writes recorded runs as the JSON files the browser viewer replays.

The viewer does no simulation and no kinematics. Everything it draws is computed here, from
the core, so what a reviewer sees in the browser is what the tests measured.
"""

import json
from pathlib import Path

import numpy as np
from stapes_core import chain_positions

from .run import Trace, run
from .scenarios import SCENARIOS

FORMAT_VERSION = 1
# One arm pose every 10 ms is smooth to the eye. The error series keep every control period,
# because a stop is decided within a few of them.
FRAME_EVERY_TICKS = 10


def _perpendicular_basis(direction: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors that, with `direction`, form a right-handed frame."""
    helper = np.array([0.0, 0.0, 1.0]) if abs(direction[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(direction, helper)
    u /= np.linalg.norm(u)
    return u, np.cross(direction, u)


def _rounded(values: np.ndarray, scale: float, digits: int) -> list:
    return np.round(values * scale, digits).tolist()


def viewer_document(trace: Trace) -> dict:
    """Everything the viewer needs to replay `trace`, as plain JSON types."""
    scenario = trace.scenario
    tunnel = scenario.trajectory
    limits = scenario.safety
    u, v = _perpendicular_basis(tunnel.direction)
    from_entry = trace.tip_m - tunnel.entry

    frame_rows = np.arange(0, len(trace.tick), FRAME_EVERY_TICKS)
    arm = scenario.arm
    chains = np.stack([chain_positions(arm, trace.q[row]) for row in frame_rows])

    return {
        "format": FORMAT_VERSION,
        "name": trace.spec.name,
        "title": trace.spec.title,
        "description": trace.spec.description,
        "seed": trace.seed,
        "period_s": trace.period_s,
        "monitored": trace.spec.monitored,
        "thrust_n": trace.spec.thrust_n,
        "tunnel": {
            "entry_m": tunnel.entry.tolist(),
            "target_m": tunnel.target.tolist(),
            "length_mm": tunnel.length_m * 1e3,
        },
        "limits": {
            "design_target_um": 200.0,
            "corridor_radius_um": limits.corridor_radius_m * 1e6 if trace.spec.monitored else None,
        },
        "events": {
            "fault_start_tick": trace.fault_start_tick,
            "stop_tick": trace.stop_tick,
            "stop_cause": trace.cause_name if trace.stop_tick is not None else None,
            "detection_ms": trace.detection_ticks,
        },
        "summary": {
            "max_radial_um": round(trace.max_radial_m * 1e6, 1),
            "max_axis_offset_um": round(float(trace.axis_offset_m.max()) * 1e6, 1),
            "final_depth_mm": round(float(trace.depth_m[-1]) * 1e3, 3),
        },
        # One value per control period.
        "series": {
            "radial_um": _rounded(trace.radial_m, 1e6, 1),
            "axis_offset_um": _rounded(trace.axis_offset_m, 1e6, 1),
            "depth_mm": _rounded(trace.depth_m, 1e3, 4),
            "planned_depth_mm": _rounded(trace.planned_depth_m, 1e3, 4),
            # Where the tip is in the plane across the tunnel: the view down the drill.
            "across_u_um": _rounded(from_entry @ u, 1e6, 1),
            "across_v_um": _rounded(from_entry @ v, 1e6, 1),
        },
        # One arm pose per frame: the joint origins and the tool tip, in metres.
        "frames": {
            "every_ticks": FRAME_EVERY_TICKS,
            "chain_m": _rounded(chains, 1.0, 6),
        },
    }


def export_all(directory: Path, seed: int = 1) -> list[Path]:
    """Runs every named scenario and writes one file each, plus an index."""
    directory.mkdir(parents=True, exist_ok=True)
    written = []
    index = []
    for spec in SCENARIOS.values():
        document = viewer_document(run(spec, seed))
        path = directory / f"{spec.name}.json"
        # allow_nan=False: a NaN is not valid JSON, and a run that produced one is a defect to
        # hear about here, not in the browser.
        path.write_text(json.dumps(document, allow_nan=False, separators=(",", ":")) + "\n",
                        encoding="utf-8")
        written.append(path)
        index.append({
            "name": spec.name,
            "title": spec.title,
            "file": path.name,
            "stopped": document["events"]["stop_tick"] is not None,
            "monitored": spec.monitored,
        })
    index_path = directory / "index.json"
    index_path.write_text(json.dumps({"format": FORMAT_VERSION, "scenarios": index}, indent=2)
                          + "\n", encoding="utf-8")
    written.append(index_path)
    return written
