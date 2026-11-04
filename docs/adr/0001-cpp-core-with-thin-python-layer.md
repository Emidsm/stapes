# ADR 0001: C++ core with a thin Python layer

**Status:** accepted, 2026-11-03

## Context

Stapes needs fast, deterministic physics and control code, plus a convenient way to
write scenarios and analyse results.

## Decision

- All simulation, control, safety and fault-injection code is C++20.
- Python is a layer on top, through pybind11, for scenarios and analysis. The core never
  depends on it and builds without it.
- The standard stays at C++20, rather than C++23, as a conservative choice for this kind
  of software.
- The 3D viewer replays recorded traces in the browser, so it cannot affect a run.

## Alternatives considered

- **Rust:** strong safety guarantees, but it would split the code base without adding to
  what the project needs to show.
- **Python for the simulation:** too slow for a 1 kHz control loop with sub-steps, and
  harder to reason about allocation and timing.

## Consequences

Scenarios can change without recompiling, but each C++ type exposed to Python has a small
binding to maintain.
