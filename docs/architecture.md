# Architecture

Stapes simulates a 6-DOF surgical robot arm drilling a keyhole tunnel, with the control software
and an independent safety monitor running against it. This document describes how the code is
divided and why. The decisions behind it are recorded one by one in [`adr/`](adr/).

## The idea in one picture

```
                    software                 |            hardware
                                             |
   task ──► target ──► control ──────────────┼──► RobotInterface ───┐
  (plan)                  ▲                  |                      │
                          └── motor encoders ┼──────────────────────┤
                                             |                      ▼
                                             |              plant (simulated arm):
                       safety ◄── link encoders, tracker ───  dynamics, motors,
                      (monitor)              |                gearboxes, sensors
                          │                  |                      ▲
                          └── brakes ────────┼──► SafetyInterface ──┘
                                             |
                          faults: sits on this line, implementing both interfaces
```

Everything left of the line is software that could run on a real robot. Everything right of it is
the simulation. The line itself is two small abstract classes in `core/hardware`.

## Modules

Each module is a separate CMake target, and a module can only include what it links. A dependency
in the wrong direction is therefore a build error, not a convention.

| Module | Contents | Depends on |
|---|---|---|
| `base` | Simulated clock, random generators, state hash | nothing |
| `math` | Rigid transforms, rotation error, wrench | Eigen only |
| `kinematics` | Arm model, forward kinematics, Jacobian, inverse kinematics | `base`, `math` |
| `dynamics` | Mass properties, inverse and forward dynamics, mass matrix | `kinematics` |
| `hardware` | `RobotInterface` and `SafetyInterface`: the boundary | `kinematics` |
| `plant` | The simulated arm: physics, motors, gearboxes, sensors, brakes | `hardware`, `dynamics` |
| `control` | PID per joint, gravity and gearbox compensation | `hardware`, `dynamics` |
| `safety` | The safety monitor and its limits | `hardware` |
| `faults` | The fault injector | `hardware` |
| `task` | Keyhole trajectory, feed profile, the axis constraint | `kinematics` |
| `sim` | Assembles a complete run; records traces | all of the above |
| `bindings` | Python module, a translation of types only | `sim` |

`plant`, `control`, `safety` and `faults` depend on `hardware` and never on each other. That one
rule gives three properties:

- **The controller cannot tell it is simulated.** It sees `RobotInterface`: read sensors, command
  torques. It has no access to the simulator's true state, so it cannot cheat (ADR 0006).
- **The monitor shares nothing with the controller.** It does not link `control`, so it cannot
  read the controller's state, and it has its own sensors and its own way to stop (ADR 0007).
- **Faults need no hooks.** The fault injector implements both interfaces and sits between the
  two sides. The code under test runs unmodified, and the plant knows nothing of faults (ADR 0009).

## One control period

`sim::KeyholeSimulation::step` runs one period of 1 ms, always in this order:

1. **Plan.** The feed profile gives the depth for this moment; inverse kinematics, started from
   the last solution, turns the pose at that depth into joint targets. In a run that starts from
   rest, a joint-space move gives the targets until the arm is on the line, 5 mm before the
   entry (ADR 0012).
2. **Read.** The controller's readings and the monitor's readings are taken through the fault
   injector.
3. **Check.** The monitor judges this period's input. If it stops, the brakes are engaged in this
   same period and the controller is not run.
4. **Control.** Otherwise the controller computes torques from the target and the motor encoders,
   and commands them.
5. **Physics.** The plant advances ten steps of 0.1 ms and samples its sensors.

The monitor runs before the controller so that a stop takes effect in the period it is detected.

## The simulated robot

Each joint is two masses, the motor and the link, joined by a gearbox that is elastic and has
backlash (ADR 0005). The motor encoders are on the motor side, as on a real geared arm, so the
controller cannot see the gearbox flex. This single modelling choice is what makes the rest of
the project meaningful: it creates a difference between what the controller believes and what is
true, and the safety channel exists to see that difference.

Rigid-body dynamics use the recursive Newton-Euler algorithm for inverse dynamics and the
composite rigid body algorithm for the mass matrix. Integration is semi-implicit Euler, which
does not steadily add energy to a stiff spring such as a gearbox.

Sensors are modelled with their defects: encoder noise and quantisation, force sensor noise,
scale error and random-walk drift, tracker position and orientation noise.

## Determinism

The same seed gives the same run, bit for bit. A hash of the whole state is compared between
runs, between processes and, in CI, between GCC and Clang (ADR 0002). The rules that make this
true:

- Time is an integer count of fixed steps. Nothing reads the wall clock.
- Randomness comes only from `base::Rng`: xoshiro256++ with a Gaussian sampler written here,
  because `std::normal_distribution` gives different values in different standard libraries.
- Each noise source has its own random stream, so adding a sensor does not change the noise
  another sensor sees.
- One thread. Delay is modelled with fixed buffers in simulated time.
- The compiler may not fuse a multiply and an add (`-ffp-contract=off`).

The claim is limited to what is tested: a given compiler and maths library.

## Real-time discipline

No tick allocates memory. Every vector and matrix has a fixed size, delays use fixed arrays, and
the trace recorder reserves its storage before the run. A test replaces the global `operator new`
with a counter and runs complete ticks, with a fault and a stop, to prove the count stays zero.

## From a run to evidence

```
 C++ tests ──► JSON results ──► tools/traceability.py ──► traceability matrix
     │                     └──► tools/test_report.py  ──► test report, README figures
     │
 sim::TraceRecorder ──► Python (stapes) ──► JSON traces ──► viewer (browser)
                                       └──► plots ──► manual
```

- **Requirements** have IDs. Each test declares the IDs it verifies, and the matrix is built from
  the test results. A requirement with no passing test fails the build.
- **Hazards** list requirement IDs as their controls, and the same tool checks them.
- **Injected bugs.** `tools/mutation_check.py` changes one line at a time and fails if the tests
  do not notice. Line coverage shows a line ran; this shows a test would notice if it were wrong.
- **Traces.** The recorder stores the true state each tick without changing the run. The Python
  layer chooses scenarios and writes files; the viewer only replays them (ADR 0010).

## What is not modelled

- Collisions between the arm and the patient or staff during the approach from rest.
- Patient anatomy, bone and tissue. The tunnel is a given straight line, as a surgical plan would
  supply it, and a drilling load is an external force.
- Tracker latency and a tracker rate below the control rate.
- Thermal effects, cable forces, and structural flexibility of the links.
- A real-time operating system. The tick is deterministic and does not allocate, but nothing here
  measures scheduling jitter.
