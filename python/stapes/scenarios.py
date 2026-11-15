"""The named scenarios: what each run is, and what it is expected to show."""

from dataclasses import dataclass

from stapes_core import Fault, FaultKind, KeyholeScenario, without_monitor_limits

# Long enough for the reference feed to finish and settle. The C++ tests use the same length.
RUN_TICKS = 3200
FAULT_START_TICK = 800


@dataclass(frozen=True)
class ScenarioSpec:
    """One run of the reference drilling task, with at most one thing going wrong."""

    name: str
    title: str
    description: str
    fault_kind: FaultKind = FaultKind.none
    fault_joint: int = 0
    fault_delay_ticks: int = 0
    thrust_n: float = 0.0  # Force opposing the feed, built up over the first 0.4 s.
    monitored: bool = True
    expect_stop: bool = False
    ticks: int = RUN_TICKS

    @property
    def has_fault(self) -> bool:
        return self.fault_kind != FaultKind.none

    def build(self, seed: int) -> KeyholeScenario:
        scenario = KeyholeScenario.reference(seed)
        if self.has_fault:
            scenario.fault = Fault(
                kind=self.fault_kind,
                start_tick=FAULT_START_TICK,
                joint=self.fault_joint,
                delay_ticks=self.fault_delay_ticks,
            )
        return scenario if self.monitored else without_monitor_limits(scenario)


_SPECS = (
    ScenarioSpec(
        name="nominal",
        title="Nominal drilling",
        description="The reference 10 mm tunnel with no fault. The tip stays within the "
        "0.2 mm design target and the monitor stays silent.",
    ),
    ScenarioSpec(
        name="thrust-2n",
        title="Drilling against 2 N",
        description="A thrust opposing the feed, as bone resists a drill. The gearboxes flex, "
        "the motor encoders cannot see it, and the tip still holds the target.",
        thrust_n=2.0,
    ),
    ScenarioSpec(
        name="thrust-5n",
        title="Drilling against 5 N",
        description="More thrust than joint-level control can hold. The tracker sees the tip "
        "leave the line and the monitor stops the robot.",
        thrust_n=5.0,
        expect_stop=True,
    ),
    ScenarioSpec(
        name="frozen-motor-encoder",
        title="Frozen motor encoder",
        description="One motor encoder repeats its last value with a fresh timestamp. The "
        "controller is blind on that joint; the link encoder is not.",
        fault_kind=FaultKind.frozen_motor_encoder,
        fault_joint=1,
        expect_stop=True,
    ),
    ScenarioSpec(
        name="frozen-motor-encoder-unmonitored",
        title="Frozen motor encoder, monitor disabled",
        description="The same fault with every monitor limit out of reach: the baseline that "
        "shows the fault is real and the monitor is what contains it.",
        fault_kind=FaultKind.frozen_motor_encoder,
        fault_joint=1,
        monitored=False,
    ),
    ScenarioSpec(
        name="frozen-link-encoder",
        title="Frozen link encoder",
        description="A sensor on the safety channel freezes. The controller is unaffected, "
        "and the monitor notices its own sensors disagreeing.",
        fault_kind=FaultKind.frozen_link_encoder,
        fault_joint=1,
        expect_stop=True,
    ),
    ScenarioSpec(
        name="frozen-tracker",
        title="Frozen optical tracker",
        description="The tracker repeats its last pose. The arm keeps moving, so the tracker "
        "and the link encoders drift apart.",
        fault_kind=FaultKind.frozen_tracker,
        expect_stop=True,
    ),
    ScenarioSpec(
        name="stuck-actuator",
        title="Stuck actuator",
        description="One motor holds its last torque and ignores new commands.",
        fault_kind=FaultKind.stuck_actuator,
        fault_joint=1,
        expect_stop=True,
    ),
    ScenarioSpec(
        name="comms-delay",
        title="Communication delay",
        description="Control readings and commands arrive 20 ms late. The monitor refuses "
        "the stale data in the tick it appears.",
        fault_kind=FaultKind.comms_delay,
        fault_delay_ticks=20,
        expect_stop=True,
    ),
)

SCENARIOS: dict[str, ScenarioSpec] = {spec.name: spec for spec in _SPECS}
