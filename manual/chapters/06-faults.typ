#import "/template.typ": *

#chapter("Injecting faults", summary: [
  A safety monitor that has never seen a fault has not been tested.
])

== The fault injector

The fault injector implements both hardware interfaces and sits between the software and the
robot. Neither side can tell it is there. The controller and the monitor run unmodified, and the
simulated robot contains no fault logic at all.

Brake commands always pass straight through. They are the path a stop must not depend on.

#data-table(
  (auto, 1fr),
  [Fault], [What it does],
  [`frozen_motor_encoder`], [One motor encoder repeats its last value. Its timestamp stays fresh.],
  [`frozen_link_encoder`], [The same for one link encoder, on the safety channel.],
  [`frozen_tracker`], [The tracker repeats its last pose, with a fresh timestamp.],
  [`stuck_actuator`], [One motor holds its last torque and ignores new commands.],
  [`comms_delay`], [Control readings and commands both arrive late, by a set number of ticks.],
)

#why[
  A frozen sensor keeps a fresh timestamp because that is the harder case. A dead bus shows up as
  stale data at once. A stuck value on a live bus looks healthy, and can only be found by
  comparing it with something else.
]

Delay is modelled with fixed buffers in simulated time. There are no threads, so a delayed run is
as reproducible as any other.

== The campaign

The fault campaign runs each single-joint fault on every joint, starting at two different times,
with two seeds: 24 runs per fault. What it requires is an _outcome_, not a detection:

- the true tip and the tool axis stay within 0.5 mm of the plan in every run;
- at least three quarters of the runs end in a stop. A joint may move too little in the time left
  for its fault to matter, and such a run is harmless without a stop.

#data-table(
  (1fr, auto, auto, auto),
  [Fault], [Runs stopped], [Worst excursion], [Slowest detection],
  [Frozen motor encoder], [#measured("frozen_motor_encoder_stops") of 24],
  [#measured("frozen_motor_encoder_worst_excursion_um") µm],
  [#measured("frozen_motor_encoder_worst_latency_ms") ms],
  [Frozen link encoder], [#measured("frozen_link_encoder_stops") of 24],
  [#measured("frozen_link_encoder_worst_excursion_um") µm],
  [#measured("frozen_link_encoder_worst_latency_ms") ms],
  [Stuck actuator], [#measured("stuck_actuator_stops") of 24],
  [#measured("stuck_actuator_worst_excursion_um") µm],
  [#measured("stuck_actuator_worst_latency_ms") ms],
  [Frozen tracker], [all], [—], [#measured("frozen_tracker_worst_latency_ms") ms],
)

A long detection time is not by itself a failure. A fault on a joint that is barely moving does
no harm until the joint has to move, and the monitor stops it when it does. The excursion is the
number that matters.

== The baseline

A campaign in which nothing bad happens would pass just as well if the faults were harmless. So
the same faults are run again with every monitor limit out of reach.

#plot("containment", [
  The same frozen motor encoder with the monitor active and with its limits out of reach. The
  dashed red line is the fault; the solid red line is the stop. The vertical axis is logarithmic.
])

Without the monitor the tip leaves the line by
#measured("unmonitored_frozen_encoder_excursion_mm") mm: the controller keeps pushing a joint it
believes is not moving. This is what the monitor prevents, and the test requires it, so the
campaign cannot pass with faults that do nothing.

== Delay

A delay beyond two ticks is refused as stale data in the tick it appears. A delay of one or two
ticks is tolerated, and the tip stays within 0.2 mm. The control loop itself survives five ticks
in each direction, at #measured("delay_margin_max_radial_um") µm, which is more than twice what
the monitor will ever let it run with.
