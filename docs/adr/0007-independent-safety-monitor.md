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

Limits are set from the margin measured on a run with no fault, the worst of 60 seeds.

| Check | Worst with no fault | Limit |
|---|---|---|
| Tracker against link encoders | 157 um | 0.3 mm |
| Tracked tip distance from the line | 183 um | 0.3 mm |
| Motor encoder against link encoder | 0.53 mrad | 1.0 mrad |
| Link against target | 1.15 mrad | 1.8 mrad |

## Consequences

- The link encoders and tracker are simulated with illustrative noise. Real figures would change
  these limits, which is why they live in one configuration rather than in the monitor.
- A fault on a joint that is not moving may go unnoticed until it matters. Requirement SRS-SAF-019
  is therefore stated in terms of harm, the true tip staying within 0.5 mm of the plan, and not
  only in terms of detection.
- A real tracker is slower than 1 kHz and has latency. This one is neither, so the detection times
  measured here are optimistic.
- The force sensor has no second channel, so a frozen force reading is not detected.
