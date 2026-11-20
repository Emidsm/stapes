#import "/template.typ": *

#chapter("The safety monitor", summary: [
  A second opinion that shares nothing with the first, and has the last word.
])

== Independence

A monitor that reads what the controller reads shares the controller's blind spots. A frozen
motor encoder looks, to both, like a joint standing perfectly still. So the monitor has:

- *its own sensors*: the link encoders and the optical tracker;
- *its own copy of the arm's geometry*, and copies of its inputs, never references to the
  controller's data;
- *its own library*, which links neither the controller nor the simulator. A test also scans the
  module's sources for any include of either;
- *its own actuator*: the brakes, which need no cooperation from the controller.

== The checks

Once per control period the monitor runs every check, in a fixed order. The first that fails
decides the reported cause.

#data-table(
  (auto, 1fr, auto),
  [Cause], [The monitor stops when], [Acts],
  [`non_finite_input`], [any reading or the target contains NaN or infinity], [at once],
  [`stale_data`], [a sample on either channel is older than 2 ticks, or stamped in the future], [at once],
  [`joint_limit`], [a link is within 0.02 rad of the end of its range], [at once],
  [`joint_velocity`], [a link moves faster than 0.2 rad/s between two readings], [at once],
  [`tool_force`], [the force on the tool exceeds 20 N], [after 3 checks],
  [`corridor`], [from 2 mm before the entry inward, the tracked tip is more than 0.3 mm from the line or the tool axis more than 0.3 mm from the entry; anywhere, the tip is more than 0.3 mm past the target], [after 3 checks],
  [`tracker_disagreement`], [the tracker and the link encoders disagree about the tip by more than 0.3 mm], [after 3 checks],
  [`encoder_disagreement`], [a motor encoder and its link encoder disagree by more than 1 mrad], [after 3 checks],
  [`tracking_error`], [a link is more than 1.8 mrad from where the controller was told to put it], [after 3 checks],
)

=== Why some checks wait

Conditions that cannot be noise stop at once: a NaN is never noise. Conditions read from noisy
sensors must fail three times in a row, so that one outlier does not stop a procedure. Each of
those checks keeps its own count. Two different conditions alternating are not one condition
lasting twice as long, and a condition that comes and goes does not reset the count of another.

=== Why cross-checks

Three of the checks compare two independent sensors. This finds a frozen sensor whichever of the
two froze, and it does not depend on guessing what a fault looks like. Looking for a reading
that repeats would not work: at rest, a quantised encoder repeats its value in normal operation.

=== Why the corridor is three tests

The tip's distance from the line and the shaft's distance from the entry look alike, and they
are independent. A tool pivoting about the entry keeps its shaft in the keyhole while its tip
leaves the line. A tilted tool with its tip on the line has its shaft outside the keyhole. Only
the far end has a depth limit: backing out is the safe direction.

=== Where the corridor starts

A run can start from a rest pose, centimetres from the entry, with the tool pointing elsewhere.
The two distance checks would stop it at once. So they apply only from an entry guard, 2 mm
before the entry, inward. Outside the guard the tip is in free space and the other checks still
hold it: speed, tracking, the sensors against each other.

The monitor is not told which phase the run is in. It decides from the tracked tip alone. A
phase flag from the controller would be a second way into the monitor, and a controller wrong
about its own phase could turn the corridor off while drilling. With the guard, being near the
entry is exactly what turns the corridor on. The approach is planned to arrive on the line 5 mm
out, so the tip is already on it when the guard starts (ADR 0012).

== The limits and their margins

Each limit was set from what a healthy run looks like through the monitor's own sensors, the
worst of 60 seeds. A test measures this on every build, so a change that uses up a margin shows
up there before it shows up as a false stop.

#data-table(
  (1fr, auto, auto),
  [Check], [Worst healthy value], [Limit],
  [Tracker against link encoders], [#measured("margin_tracker_against_links_um") µm], [300 µm],
  [Tracked tip to the planned line], [#measured("margin_tracked_tip_off_line_um") µm], [300 µm],
  [Tracked tool axis to the entry], [#measured("margin_tracked_shaft_at_entry_um") µm], [300 µm],
  [Motor encoder against link encoder], [#measured("margin_motor_against_link_mrad", digits: 2) mrad], [1.0 mrad],
  [Link against target], [#measured("margin_link_against_target_mrad", digits: 2) mrad], [1.8 mrad],
  [Tool force], [#measured("margin_tool_force_n", digits: 2) N], [20 N],
)

== A stop

#stop-note[
  When a check fails, the monitor engages the brakes in the same control period, and the
  controller is not run again. The stop is _latched_: the monitor keeps reporting its first
  cause, whatever later input says.
]

A reset is refused while any condition is still present. When it succeeds, the monitor forgets
its counts and its earlier reading, because the robot may have been moved by hand while stopped,
and that is not a velocity.

The monitor's two functions do not allocate memory, do not throw, and do not read a clock. Its
constructor is the only place that can fail, and it rejects any limit that is not positive and
finite.
