"""Scenarios and analysis for the Stapes simulation.

The simulation itself is the C++ core, reached through `stapes_core`. This package only
decides which runs to make and what to do with their results.
"""

from stapes_core import (
    JOINT_COUNT,
    ArmModel,
    Fault,
    FaultKind,
    KeyholeScenario,
    KeyholeSimulation,
    StateHash,
    StopCause,
    TraceRecorder,
    __version__,
    chain_positions,
    compiler,
    forward_kinematics,
    reference_home,
    stop_cause_name,
    without_monitor_limits,
)

from .run import Trace, run
from .scenarios import SCENARIOS, ScenarioSpec

__all__ = [
    "JOINT_COUNT",
    "SCENARIOS",
    "ArmModel",
    "Fault",
    "FaultKind",
    "KeyholeScenario",
    "KeyholeSimulation",
    "ScenarioSpec",
    "StateHash",
    "StopCause",
    "Trace",
    "TraceRecorder",
    "__version__",
    "chain_positions",
    "compiler",
    "forward_kinematics",
    "reference_home",
    "run",
    "stop_cause_name",
    "without_monitor_limits",
]
