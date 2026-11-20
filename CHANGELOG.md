# Changelog

## 1.0.0 (2026-11-19)

The first complete version: a simulated robot, its control software, an independent safety
monitor, fault injection, and the tooling that turns test results into evidence.

### Simulation

- Simulated time as an integer tick count, a seeded random generator with its own Gaussian
  sampler, and a hash of the whole state for comparing runs bit by bit.
- A 6-DOF arm: forward kinematics, the geometric Jacobian, and Levenberg-Marquardt inverse
  kinematics with joint limits.
- Rigid-body dynamics: recursive Newton-Euler, the composite rigid body mass matrix, and forward
  dynamics.
- Two-mass joints with friction, elastic gearboxes and backlash. Motor encoders, link encoders,
  a force sensor with drift, an optical tracker, and brakes.

### Control and safety

- A hardware boundary of two interfaces, which the controller and the monitor cannot see past.
- Joint control: PID with anti-windup, gravity compensation and gearbox compensation.
- The keyhole task: a straight tunnel, a feed profile, and one constraint shared with a remote
  centre of motion.
- An approach from a rest pose: a joint-space move with a quintic time scaling, ending on the
  line 5 mm before the entry.
- A safety monitor with its own sensors and its own stop: four immediate checks, five debounced
  checks, a latched stop and a guarded reset. Its corridor starts at an entry guard, judged from
  the tracked tip alone.
- Fault injection: frozen motor encoder, frozen link encoder, frozen tracker, stuck actuator and
  communication delay.

### Evidence

- Requirements with identifiers, a traceability matrix and a test report generated from test
  results, and a hazard list whose controls are checked by the same tool.
- Injected bugs, one line at a time, that the tests must catch.
- A trace recorder, Python bindings and named scenarios.
- A browser viewer that replays recorded runs, with a camera that follows the tool and optional
  sound, and an operation manual built from the test results.

### Known limits

- Results are bit-exact for a given compiler and maths library only.
- The 0.2 mm accuracy holds for a drilling thrust up to 2 N.
- The simulated tracker has no latency, and the single force sensor has no second channel.
- Collisions between the arm and its surroundings during the approach are not modelled.
