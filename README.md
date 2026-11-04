# Stapes

A deterministic simulation environment for a 6-DOF surgical robot arm, in C++20.

Named after the stapes, the smallest bone in the human body, which sits beside the round
window of the cochlea.

> **Work in progress.** The foundation is in place: simulated time, reproducible
> randomness, and forward and inverse kinematics. Dynamics, actuator and sensor models,
> the safety monitor and fault injection come next.

The anatomy and tissue models are illustrative, and the reference arm is not the
geometry of any real robot. This is an engineering project, not a medical device.

## Build and test

Requires CMake 3.28+, Ninja and a C++20 compiler. Dependencies are downloaded and
checked against pinned hashes.

```sh
cmake --workflow --preset debug      # build and run the tests
cmake --workflow --preset release
cmake --workflow --preset sanitize   # with AddressSanitizer and UBSan
cmake --workflow --preset coverage   # with GCC coverage instrumentation
```

## Verification

Each requirement in [`docs/requirements/requirements.toml`](docs/requirements/requirements.toml)
has an ID, and each test declares the IDs it verifies. The traceability tool builds the
matrix from the test results and fails if a requirement has no passing test:

```sh
python3 tools/traceability.py \
  --requirements docs/requirements/requirements.toml \
  --results build/debug/test-results \
  --output build/debug/traceability.md
```

Coverage shows that a line ran, not that a test would notice if it were wrong. To check
that, this tool injects known bugs one at a time and fails if any goes undetected:

```sh
python3 tools/mutation_check.py
```

## Layout

| Path | Contents |
|---|---|
| `core/base` | Simulated clock, random generators, state hash |
| `core/math` | Rigid transforms and rotation error |
| `core/kinematics` | Arm model, forward kinematics, Jacobian, inverse kinematics |
| `tests` | Unit and property tests, determinism checks |
| `benchmarks` | Timing of the kinematics functions |
| `tools` | Traceability, bug injection, reference values |
| `docs/adr` | Architecture decision records |
| `docs/requirements` | Software requirements |

## License

All rights reserved: you may read, build and run this code to evaluate it. See
[`LICENSE`](LICENSE).
