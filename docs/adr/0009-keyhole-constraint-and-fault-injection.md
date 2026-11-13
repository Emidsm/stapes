# ADR 0009: One constraint for two scenarios, and faults as a wrapper

**Status:** accepted, 2026-11-04

## Context

The main scenario is keyhole drilling along a straight planned tunnel. A laparoscopic remote
centre of motion is a secondary one. The faults to inject are a frozen sensor, a stuck actuator
and a communication delay, and the code under test must run unmodified when they occur.

## Decision

- **One constraint.** Both scenarios require the tool axis to pass through a fixed point, so
  `task::axis_distance` measures both. They differ only in how the tip moves: along the axis for
  a tunnel, across it for a pivot. Poses are built by the smallest rotation from a reference, so
  the free roll about the tool axis does not wander.
- **Task space goes through inverse kinematics each tick**, warm-started from the last solution.
  The whole tunnel is walked once before any motion, and a plan that cannot be reached is refused
  with the arm unmoved.
- **A tunnel tilted out of the arm's plane.** The first reference tunnel lay in that plane, so
  three joints did not move and a fault on them showed nothing. Now every joint has to move.
- **Faults as a wrapper.** `faults::FaultInjector` implements the same interfaces as the robot and
  sits between it and the software, so neither side can tell it is there. Delays use fixed buffers
  in simulated time, without threads or allocation. Brakes pass straight through it, since they
  are the path a stop must not depend on.
- **A frozen sensor keeps a fresh timestamp**, the harder case. A dead bus would show up as stale
  data at once; a stuck value on a live bus can only be found by comparing it with something.

## Evidence

Measured on the reference arm on 2026-11-04. The test suite records the current values.

| Scenario | Result |
|---|---|
| Tip speed 2 mm/s around a fulcrum 40 mm behind the tip | 151 um axis offset |
| The same at 4 mm/s | 280 um |
| The same at 8 mm/s | 389 um |
| Tunnel feed under a thrust of 1 N, built up over 0.4 s, worst of 5 seeds | 132 um from the line |
| The same at 2 N | 195 um |
| The same at 3 N | 281 um, outside the 0.2 mm target, stopped in 1 run of 5 |
| The same at 5 N | 343 um, stopped by the monitor in all 5 runs |

## Consequences

- Stated capability is limited: 0.2 mm holds for a thrust up to 2 N, with only 5 um to spare, and
  for a pivot at 2 mm/s. Between 2 N and 5 N the tip can be outside the 0.2 mm target without a
  stop, since the monitor's corridor is 0.3 mm. The cause is the one ADR 0006 predicted: a force
  bends the gearboxes, and motor encoders cannot see it. The tracker sees it, but only the monitor
  reads it. Correcting the tip from the tracker in closed loop is the next step for accuracy under
  load, and is not done.
- There are no physical joint stops. A run with the monitor's limits disabled can pass a joint
  limit, so excursions recorded for those baseline runs are only meaningful as "far out of bounds".
