# ADR 0006: Hardware boundary and gearbox compensation

**Status:** accepted, 2026-11-03

## Context

A simulation environment is most useful as part of a robot's software framework: the same
control code should be able to run against the simulator or the real robot.

With encoders on the motor side, a first closed-loop measurement had the motors on target to
within 11 urad while the links sagged by up to 4.3 mrad under gravity. Part of this came from
gearbox stiffness set too low. Even at realistic stiffness, uncompensated control leaves the tool
tip outside the 0.2 mm design target.

## Decision

- `hardware::RobotInterface` is the only thing control code can see: sensor readings in, torque
  commands out. `control` and `plant` both depend on it, and neither links the other, so the
  build itself stops control code from reading the simulator's true state.
- `JointController` aims each motor past its target by the deflection it predicts:
  `load / stiffness + backlash * tanh(load / blend)`, using gravity from its own model.
- The controller's model and gearbox figures are its own estimates, separate from the plant's
  parameters, so the two can differ as they would on real hardware.

## Evidence

Measured on the reference arm on 2026-11-03. The test suite records the current values in its
results.

| Case | True tip error |
|---|---|
| Uncompensated, holding a pose | 274 um |
| Compensated, holding a pose | 0.24 um |
| Compensated, 5 % mass error and 10 % stiffness error in the model | 28 um |
| Compensated, under an unmodelled 5 N force at the tip | 108 um |

## Consequences

- Accuracy depends on the gearbox estimates, and on knowing the load. The PID integral corrects
  torque errors at the motor but cannot see an error in the predicted deflection, since that lies
  beyond the encoder.
- An unmodelled external force moves the tip with no change in any encoder reading. That is
  a limit of joint-level sensing, and the reason to add external position sensing in M2.
