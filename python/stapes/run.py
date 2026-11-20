"""Runs a scenario and returns what happened as arrays."""

import math
from dataclasses import dataclass

import numpy as np
from stapes_core import (
    KeyholeScenario,
    KeyholeSimulation,
    StopCause,
    TraceRecorder,
    stop_cause_name,
)

from .scenarios import ScenarioSpec

THRUST_RAMP_TICKS = 400
# After the feed has finished, how long a run that starts from rest goes on, so it settles.
SETTLE_S = 0.5


@dataclass(frozen=True)
class Trace:
    """One recorded run. Every array has one row per control period."""

    spec: ScenarioSpec
    seed: int
    scenario: KeyholeScenario
    tick: np.ndarray
    planned_depth_m: np.ndarray
    depth_m: np.ndarray
    radial_m: np.ndarray
    axis_offset_m: np.ndarray
    q: np.ndarray  # True link angles, one column per joint.
    target: np.ndarray
    tip_m: np.ndarray
    tool_axis: np.ndarray
    fault_active: np.ndarray
    brakes_engaged: np.ndarray
    approaching: np.ndarray
    cause: StopCause
    stop_tick: int | None

    @property
    def period_s(self) -> float:
        return self.scenario.safety.period_s

    @property
    def time_s(self) -> np.ndarray:
        return self.tick * self.period_s

    @property
    def cause_name(self) -> str:
        return stop_cause_name(self.cause)

    @property
    def fault_start_tick(self) -> int | None:
        return int(self.scenario.fault.start_tick) if self.spec.has_fault else None

    @property
    def inside_guard(self) -> np.ndarray:
        """The samples where the monitor holds the tip to the line: from the entry guard inward.

        Before that, during an approach, the tip is centimetres off the line by design, and a
        distance from the line means nothing.
        """
        return self.depth_m > -self.scenario.safety.entry_guard_m

    @property
    def max_radial_m(self) -> float:
        inside = self.radial_m[self.inside_guard]
        return float(inside.max()) if inside.size else 0.0

    @property
    def max_axis_offset_m(self) -> float:
        inside = self.axis_offset_m[self.inside_guard]
        return float(inside.max()) if inside.size else 0.0

    @property
    def approach_end_tick(self) -> int | None:
        """The first tick of the feed, in a run that starts from rest."""
        if not self.spec.approach:
            return None
        feeding = np.flatnonzero(~self.approaching)
        return int(self.tick[feeding[0]]) if feeding.size else None

    @property
    def detection_ticks(self) -> int | None:
        """Control periods from the fault starting to the monitor stopping the robot."""
        if self.stop_tick is None or self.fault_start_tick is None:
            return None
        return self.stop_tick - self.fault_start_tick


def run_ticks(spec: ScenarioSpec, scenario: KeyholeScenario, sim: KeyholeSimulation) -> int:
    """How many control periods a run of `spec` lasts."""
    if not spec.approach:
        return spec.ticks
    end_s = sim.feed_start_s + sim.feed_duration_s + SETTLE_S
    return math.ceil(end_s / scenario.safety.period_s)


def run(spec: ScenarioSpec, seed: int = 1) -> Trace:
    """Runs `spec` from start to finish, recording every control period."""
    scenario = spec.build(seed)
    sim = KeyholeSimulation(scenario)
    ticks = run_ticks(spec, scenario, sim)
    recorder = TraceRecorder(ticks)
    against_feed = -scenario.trajectory.direction
    wrench = np.zeros(6)
    for tick in range(ticks):
        if spec.thrust_n != 0.0:
            ramp = min(1.0, tick / THRUST_RAMP_TICKS)
            wrench[:3] = against_feed * (spec.thrust_n * ramp)
            sim.set_external_wrench(wrench)
        sim.step()
        recorder.record(sim)
    if recorder.dropped != 0:
        raise RuntimeError(f"trace is incomplete: {recorder.dropped} samples dropped")

    return Trace(
        spec=spec,
        seed=seed,
        scenario=scenario,
        tick=recorder.tick().astype(np.int64),
        planned_depth_m=recorder.planned_depth_m(),
        depth_m=recorder.depth_m(),
        radial_m=recorder.radial_m(),
        axis_offset_m=recorder.axis_offset_m(),
        q=recorder.q(),
        target=recorder.target(),
        tip_m=recorder.tip_m(),
        tool_axis=recorder.tool_axis(),
        fault_active=recorder.fault_active().astype(bool),
        brakes_engaged=recorder.brakes_engaged().astype(bool),
        approaching=recorder.approaching().astype(bool),
        cause=sim.cause,
        stop_tick=sim.stop_tick,
    )
