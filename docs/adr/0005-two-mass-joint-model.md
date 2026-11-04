# ADR 0005: Two-mass joint model with motor-side encoders

**Status:** accepted, 2026-11-03

## Context

The offer this project answers asks for simulated physical actuators and sensors. A plant that
applies commanded torque straight to the link would make control look easier than it is, and
would have no place for friction, backlash or gearbox flex to act.

## Decision

- Each joint has two masses, the motor and the link, coupled by an elastic gearbox with a dead
  zone. The links follow rigid-body dynamics: recursive Newton-Euler for the torques and the
  composite rigid body algorithm for the mass matrix.
- Friction acts on the motor side, combining Coulomb, Stribeck and viscous terms. The jump at
  zero speed is smoothed with `tanh`, so the arm creeps very slowly under a load instead of
  sticking exactly.
- Encoders read the motor, as on most geared arms, so the controller cannot see backlash or
  gearbox flex.
- Physics runs at 10 kHz with semi-implicit Euler, and control at 1 kHz.
- Physical joint stops are not modelled. Limits are the safety monitor's job, in M2.

## Alternatives considered

- **Single mass per joint:** simple and fast, but with no backlash and no difference between
  what the encoder reads and where the link is, which is the most interesting failure here.
- **Runge-Kutta for the plant:** more accurate on smooth motion, but gear contact is not
  smooth, and it would cost four dynamics evaluations per step.

## Evidence

Conservation of energy is checked by convergence rather than by a fixed tolerance. Drift falls
at the integrator's order as the step shrinks, which most errors in the dynamics would not do.

Energy has a blind spot: the gyroscopic term `w x Iw` does no work, so removing it conserves
energy just as well. Bug injection exposed this, and that term now has a hand-calculated test.

## Consequences

The plant is stiff, so the physics step cannot be raised freely. Parameters are illustrative:
stiffness is in the range of published harmonic-drive figures, and nothing is fitted to a real
robot.
