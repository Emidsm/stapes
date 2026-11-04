# ADR 0003: URDF-style kinematic model

**Status:** accepted, 2026-11-03

## Context

The arm's geometry has to be described in a form used by forward kinematics, the Jacobian
and, later, rigid-body dynamics.

## Decision

Each joint is an origin transform from its parent, plus a unit axis in its own frame, as in
URDF.

## Alternatives considered

- **Denavit-Hartenberg:** compact, but frames are constrained in ways that make parameters
  harder to read, and nearly parallel axes make it ill-conditioned.
- **Product of exponentials:** elegant, but a further step away from URDF, which is what
  robot descriptions commonly use.

## Consequences

- The geometry reads directly from the arm's dimensions, and a URDF importer would be a
  small addition.
- Six parameters are used where four could do, which costs nothing here.
- The Jacobian comes straight from the frames: each column is `axis x (tip - joint)` for
  linear velocity, and `axis` for angular velocity.
