# Software requirements specification

## Purpose

This document introduces the requirements of Stapes. The requirements themselves are in
[`requirements/requirements.toml`](requirements/requirements.toml), one entry each, because that
file is what the tools read: a requirement that existed only in prose could not be checked.

The structure is **after** IEC 62304, the standard for medical device software life cycles: each
requirement is identified, verifiable, and traced to the tests that verify it. No compliance with
that standard is claimed. Stapes is an engineering project, not a medical device.

## Scope

Stapes is a simulation environment for a 6-DOF robot arm that drills a straight keyhole tunnel.
It contains:

- a simulated robot, with rigid-body dynamics, geared actuators and imperfect sensors;
- control software that sees the robot only through a hardware interface;
- a safety monitor, independent of the controller, that can stop the robot;
- a fault injector, to test both against failures;
- tools that turn test results into a traceability matrix and a test report.

No physical robot is connected. The arm's geometry, the anatomy and the loads are illustrative.

## Design targets

Published work on robotic cochlear implantation reports robotic drilling with an accuracy of
about 0.2 mm, with safety mechanisms that predicted the proximity of nerves to within 0.1 mm, and
describes three ways of sensing: optical tracking, a sensor of the bone's resistance to drilling,
and a nerve stimulation probe [1]. An earlier laboratory study measured 0.15 ± 0.08 mm at the
target [2]. Stapes uses these figures as design targets. They are not claims about any real
system, and Stapes models only the first two kinds of sensing: a tracker, and a force sensor.

| Quantity | Target | Where it is required |
|---|---|---|
| True tip to the planned line, in normal operation | 0.2 mm | SRS-KEY-006, SRS-KEY-007 |
| True tip to the planned line, under any injected fault | 0.5 mm | SRS-SAF-019, SRS-SAF-021 |
| Tip motion after a safety stop | 0.2 mm | SRS-SAF-022 |
| True tip to the planned line, from the entry guard inward, after an approach from rest | 0.2 mm | SRS-KEY-011 |

## How a requirement is written

```toml
[[requirement]]
id = "SRS-SAF-004"
statement = "A sample on either channel older than the allowed age, or stamped in the future, shall stop at once."
code = ["core/safety/src/safety_monitor.cpp"]
```

- **`id`** never changes and is never reused. The middle part names the area.
- **`statement`** says what shall be true in a way a test can decide. It gives numbers where
  numbers matter.
- **`code`** lists where the requirement is implemented.
- **`layer`** is optional. `layer = "python"` marks a requirement verified by the Python tests,
  which only the `python` build runs.

A test declares what it verifies:

```cpp
TEST(SafetyMonitor, ASampleFromTheFutureIsNotTrusted) {
  stapes::test::verifies("SRS-SAF-004");
  ...
}
```

`tools/traceability.py` then builds the matrix from the test results. It fails if a requirement
has no test, if a test names a requirement that does not exist, if a test declares none, or if a
linked test did not pass.

## Areas

| Area | Subject |
|---|---|
| SIM | Simulated time |
| DET | Determinism: random generators, state hash, bit-identical runs |
| KIN | Kinematics: forward, Jacobian, inverse, joint limits |
| DYN | Rigid-body dynamics |
| ACT | Actuator models: friction, backlash, torque limits |
| SEN | Sensor models: encoders, force sensor, optical tracker |
| ARM | The simulated arm as a whole, its two sensor channels and its brakes |
| CTL | Control: PID, compensation, closed-loop accuracy, delay margin |
| KEY | The keyhole task: constraint, trajectory, feed, accuracy on the line |
| FLT | Fault injection |
| SAF | The safety monitor, and the outcome of each fault with it in the loop |
| RT | Real-time discipline: no allocation in a tick |
| TRC | Trace recording |
| PY | The Python layer: bindings, scenarios, viewer export |

## Rules for changing requirements

- A requirement is never weakened to make a failing test pass before the cause of the failure is
  understood. ADR 0008 records a case: a failing delay test exposed fragile controller gains, and
  the gains were changed, not the requirement.
- A new piece of logic gets a requirement, a test, and an injected bug in
  `tools/mutation_check.py` that the test must catch.
- Numbers quoted in the README and the manual come from the generated test report.

## References

1. S. Weber et al., "Instrument flight to the inner ear", *Science Robotics* 2(4), eaal4916, 2017.
   doi:10.1126/scirobotics.aal4916
2. B. Bell et al., "In vitro accuracy evaluation of image-guided robot system for direct cochlear
   access", *Otology & Neurotology* 34(7), 1284-1290, 2013.

## Related documents

- [Architecture](architecture.md)
- [Risk analysis](risk-analysis.md), whose controls are requirement IDs from this specification
- [Architecture decision records](adr/)
- The traceability matrix and the test report, generated by CI on every push
