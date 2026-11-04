# ADR 0004: Levenberg-Marquardt inverse kinematics with restarts

**Status:** accepted, 2026-11-03

## Context

Inverse kinematics has to work near singularities, respect joint limits, stay deterministic,
avoid allocation, and never report success falsely.

## Decision

- **Solver:** Levenberg-Marquardt on a cost combining position and weighted orientation
  error. A step is only accepted if it lowers the cost, so the cost never increases.
- **Joint limits:** candidates are clamped to the limits, and a joint on a limit that is
  being pushed further out loses its Jacobian column, so the other joints take up the error.
- **Stopping:** it reports `converged`, `max_iterations`, `stalled` or `invalid_input`. The
  caller always gets the real remaining error.
- **Far seeds:** `solve_ik_multistart` restarts from random configurations, drawn from a
  generator the caller supplies, so it stays deterministic.

## Alternatives considered

- **Closed-form solution:** possible for a spherical wrist, and faster, but it would tie the
  solver to one geometry. Stapes may yet add one if the planner needs every solution branch.
- **Damped least squares with a fixed damping schedule:** simpler, but has no guarantee that
  an iteration improves anything.

## Evidence

Measured on the reference arm over 5000 random reachable poses, starting from home, on
2026-11-03. The test suite records the current values in its results.

| Method | Poses reached |
|---|---|
| One start, 200 iterations | 3295 (65.9 %) |
| One start, 2000 iterations | 3300 (66.0 %) |
| 8 starts | 4757 (95.1 %) |
| 32 starts | 4993 (99.9 %) |

More iterations barely help, so these failures are local minima and not slow convergence.
At the failures, the wrist joints are the ones most often found on their limits. Poses within
0.5 rad per joint of the seed were all reached with one start, and that is the case that
matters while tracking a trajectory.

The stop on negligible improvement changed no success count over 5000 attempts, for targets
both reachable and 0.1 m out of reach. It reduced their solve time by 33 % and 46 %, which
matters because multistart spends most of its time in failed attempts.

## Consequences

- Finding a first configuration can take several starts, but it is bounded and deterministic.
- Tracking uses the plain solver, seeded with the previous solution.
- The solver cannot find a solution on another branch by itself. A planner that needs a
  specific branch has to seed it accordingly.
