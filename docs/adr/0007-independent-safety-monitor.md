# ADR 0007: An independent safety monitor with its own sensors

**Status:** accepted, 2026-11-04

## Context

A monitor that reads only what the controller reads shares the controller's blind spots. On this
arm the motor encoders cannot see gearbox flex, and a frozen encoder looks like a perfectly still
joint. If the monitor trusted that same value, the fault most in need of catching would be the
one it could not see.

## Decision

- **A second channel.** `hardware::SafetyInterface` has its own sensors: link encoders on the
  output of each gearbox, and an optical tracker that observes the tool from outside the arm. It
  also has its own way to stop, the brakes, which needs no cooperation from the controller. The
  force sensor is shared, because the robot has only one; this is documented in the interface.
- **No shared state.** The monitor receives copies of its inputs in `SafetyInput`. The `safety`
  target links neither `control` nor `plant`, so the build rejects any attempt to reach them.
- **Checks in a fixed order**, the first failure deciding the cause. Conditions that cannot be
  noise stop at once: non-finite input, stale data, joint limits and link speed. Conditions read
  from noisy sensors need three consecutive failures, each with its own count.
- **Cross-checks rather than plausibility checks.** Comparing two independent sensors finds a
  frozen sensor whichever one froze. Looking for a reading that repeats would not work: at rest,
  a quantised encoder repeats its value in normal operation.
- **Latched stops.** Once stopped, the monitor keeps its first cause until a reset, and a reset
  is refused while any condition persists.

Limits are set from the margin measured on a run with no fault, the worst of 60 seeds. The table
was measured again on 2026-11-14 with the finished monitor in the loop, and
`KeyholeDrilling.AHealthyRunKeepsAMarginToEveryMonitorLimit` now measures it on every build and
fails if a margin is used up.

| Check | Worst with no fault | Limit |
|---|---|---|
| Tracker against link encoders | 157 um | 0.3 mm |
| Tracked tip distance from the line | 183 um | 0.3 mm |
| Tracked tool axis distance from the entry | 179 um | 0.3 mm |
| Motor encoder against link encoder | 0.53 mrad | 1.0 mrad |
| Link against target | 1.17 mrad | 1.8 mrad |
| Tool force | 0.27 N | 20 N |

## Implementation notes

- **Counts are updated before any decision.** Each debounced check has its own counter, and all
  five are brought up to date before the monitor looks for one that has reached the limit. A
  condition that comes and goes therefore cannot reset or hide the history of another.
- **The corridor is three tests on the tracked pose:** the tip's distance from the planned line,
  the tool axis's distance from the entry point, and depth past the target. The first two look
  alike but are independent. A tool pivoting about the entry keeps its shaft in the keyhole while
  its tip leaves the line; a tilted tool with its tip on the line has its shaft outside the
  keyhole. Removing the first was tried as an injected bug and survived, because every test
  pose that failed it also failed the second. A pose pivoting about the entry now separates them.
- **The geometry is repeated, not shared.** `task::axis_distance` computes the same distance for
  the planner. The monitor does not call it, because linking `task` would give the monitor a
  dependency on the controller's side of the system.
- **Sample age uses unsigned subtraction.** A corrupted timestamp can be anywhere in the range of
  a 64-bit integer, and `now - stamp` on signed integers can overflow, which is undefined
  behaviour. A stamp in the future is refused first; for the rest the difference always fits in
  an unsigned integer.

## Consequences

- The link encoders and tracker are simulated with illustrative noise. Real figures would change
  these limits, which is why they live in one configuration rather than in the monitor.
- A fault on a joint that is not moving may go unnoticed until it matters. Requirement SRS-SAF-019
  is therefore stated in terms of harm, the true tip staying within 0.5 mm of the plan, and not
  only in terms of detection.
- A real tracker is slower than 1 kHz and has latency. This one is neither, so the detection times
  measured here are optimistic.
- The force sensor has no second channel, so a frozen force reading is not detected.
