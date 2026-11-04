# ADR 0002: Determinism strategy

**Status:** accepted, 2026-11-03

## Context

Verification depends on reproducing a run exactly: a failure seen once has to be
reproducible, and a behaviour change has to be visible as a changed result.

## Decision

- **Time** is an integer tick count. Floating-point accumulation drifts, and a test
  demonstrates this.
- **Randomness** comes from our own xoshiro256++ and polar-method Gaussian sampler.
  `std::mt19937_64` output is standardised, but `std::normal_distribution` is not, and
  standard libraries return different values for it.
- **Noise sources** each own a stream, so adding a sensor does not change the noise that
  the existing sensors receive.
- **Execution** is single-threaded. Communication delay is modelled by message queues in
  simulated time.
- **Compiler flags:** `-ffp-contract=off` prevents the compiler from fusing a multiply and
  an add, which rounds differently. There is no `-march=native` and no `-ffast-math`.
- **Evidence:** runs are reduced to an FNV-1a hash of the exact bits. Results are compared
  by hash, in-process, across processes, and between GCC and Clang.

## Limits of this guarantee

`std::log`, `std::sin` and similar functions are not required to be correctly rounded, so
their last bit can differ between C libraries. Stapes claims bit-exact results only for
the combinations that CI compares. The integer generators and `uniform01()` do not depend
on the C library.

## Consequences

Single-threaded execution limits how far a run can be parallelised. Separate runs can
still execute in parallel, which is what parameter sweeps need.
