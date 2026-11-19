# ADR 0012: An approach from rest, and a corridor that starts at an entry guard

**Status:** accepted, 2026-11-18

## Context

Every run started with the tool already at the tunnel's entry, on the line. That is the
critical phase, but it hides how the arm gets there, and the replay showed an arm that hardly
moved. A run that starts from a rest pose needs two things the system did not have: a way to
move the arm to the line, and a corridor check that does not stop it on the way. On the way,
the tip is centimetres off the line and the tool points elsewhere.

## Decision

- **A joint-space move, then the existing feed.**
  - `task::JointMove` takes every joint along a straight line in joint space, with the quintic
    time scaling 10s³ − 15s⁴ + 6s⁵. Velocity and acceleration are zero at both ends, so the
    arm starts and arrives at rest with no step in torque.
  - The move ends 5 mm before the entry, on the line, with the tool already along the tunnel.
    The arm waits there 0.5 s, then feeds in at the usual speed.
  - Joint space is used rather than a straight line in task space because, far from the
    patient, the path of the tip does not matter. What matters is where the move ends and how
    fast the joints go.
- **The corridor's distance checks start at an entry guard, judged from the tracked tip.**
  - The monitor applies the radial and shaft checks only once the tracked tip is less than
    `entry_guard_m` (2 mm) before the entry. Further out it is in free space. The depth limit
    past the target applies everywhere.
  - The monitor is not told which phase the run is in. A phase flag from the controller or the
    simulation would be a second path into the monitor, and a controller wrong about its own
    phase could switch the corridor off while drilling. With the guard, being near the entry is
    exactly what switches the corridor on.
- **The approach arrives on the line from outside the guard.** The standoff (5 mm) is larger
  than the guard (2 mm), so the tip has been on the line for 3 mm before the radius applies.
  Before any motion, the simulation checks the move at 200 points and refuses one that would
  bring the tip inside the guard. This is the same rule as a plan that cannot be reached.
- **The same limits throughout.** The monitor's speed limit (0.2 rad/s) and tracking limit stay
  in force during the approach. The move's duration is chosen so that the fastest joint, relative
  to its own limit, peaks at half of it. Faster approaches would need limits that depend on the
  phase, which is what the guard avoids.
- **The reference scenario is unchanged.** `with_approach` adds the approach to a scenario.
  `KeyholeScenario::reference` still starts at the entry, so the fault campaign and every number
  measured from it stay comparable.

## Consequences

- The reference approach takes about 14.5 s for 18 cm of tip travel. A real system would move
  faster far from the patient. Here that would take phase-dependent limits, and those would
  bring back the hazard the guard removes.
- Collisions between the arm and the patient or staff on the way are not modelled. The move is
  checked only against the tunnel's guard (HAZ-17).
- Each fault of the campaign, injected during the approach, is stopped before the tip reaches
  the guard (SRS-SAF-025). Frozen sensors are found in tens of milliseconds and a delay in the
  same tick. A stuck actuator is slower, from 0.25 s to 1.1 s. The slowest case is one stuck
  near the end of the move: the arm is slowing down, the torque it is stuck at is close to what
  the joint needs, and the tracking error grows slowly. In that case the tip stops about 7 mm
  before the entry, outside the 2 mm guard.
