# ADR 0010: Recorded traces, and a Python layer with no behaviour of its own

**Status:** accepted, 2026-11-15

## Context

A reviewer cannot be asked to read test output to see what the simulation does. The run has to be
seen: as a plot, and as a moving arm. That needs the true state of every tick to leave the C++
core, and something to decide which runs to make.

Two things could go wrong. Recording could change the run it records. And the layer on top could
grow logic of its own, which the C++ tests and the injected bugs cannot reach.

## Decision

- **Recording is part of the core.** `sim::TraceRecorder` stores one `TraceSample` per tick: the
  true tool pose, the joint state, the plan, the distances from the plan, the stop cause, and
  whether a fault is active. It is C++, so it has C++ tests and injected bugs like everything else.
- **All storage is reserved before the run.** `record` is called from the tick loop, where
  allocation is not allowed. A full recorder does not grow and does not overwrite: it counts what
  it dropped. A ring buffer was considered and rejected, because it would silently lose the start
  of a run, and the start is where a fault begins.
- **Recording must not change the run.** A test runs the same scenario with and without a recorder
  and compares state hashes.
- **Python translates and decides, and nothing else.** `bindings/` is one file that maps types.
  The package in `python/` names scenarios, runs them and writes files. The monitor's limits are
  read-only from Python: to change one, change the C++ configuration, where its tests are.
- **The unmonitored baseline moved into the core.** `sim::without_monitor_limits` used to live in
  the tests' support code. The Python layer needs it too, and copying it would have put logic in
  Python.
- **One matrix for both languages.** The Python tests write their results in the JSON layout that
  GoogleTest uses, so `tools/traceability.py` reads both. Requirements verified from Python are
  marked `layer = "python"`, and builds that do not run Python skip them explicitly.
- **Determinism is checked across the language boundary.** A Python test steps a run with a fault
  and a stop, hashes it, and compares the hash with the one a C++ program prints for the same
  run.

## Consequences

- The `python` preset is the only build that needs Python, NumPy and pybind11. The core and its
  other presets need none of them.
- pybind11 is pinned by version and hash, like the other dependencies. NumPy, Matplotlib and
  pytest are pinned in `python/requirements.txt`.
- The complete traceability matrix can only be built by the `python` preset, since no other build
  runs every test.
- A trace of the reference run is 3200 samples of about 250 bytes, under a megabyte. Runs much
  longer than this would need sampling every few ticks, which is not implemented.
