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
- **One draw per statement.** C++ does not specify the order in which a function's
  arguments are evaluated. `Vector3d(rng.gaussian(), rng.gaussian(), rng.gaussian())`
  makes the same three draws with every compiler, but GCC and Clang put them on different
  axes. Code that draws several values does so in a loop, or in a braced list, which is
  evaluated left to right.

## How the comparison between compilers paid off

The cross-compiler check failed once the optical tracker was added: kinematics matched, the
closed loop did not. Dumping every hashed value showed that sensors and torques were
identical and only the tracker's orientation differed, by whole noise samples rather than
last bits. The tracker built its tilt from three draws in one constructor call, as above.
It now draws them in a loop, and the two compilers agree again. Nothing else in a run had
read the tilt closely enough to change a reported figure, which is why no other test
noticed.

## Limits of this guarantee

`std::log`, `std::sin` and similar functions are not required to be correctly rounded, so
their last bit can differ between C libraries. Stapes claims bit-exact results only for
the combinations that CI compares. The integer generators and `uniform01()` do not depend
on the C library.

## Consequences

Single-threaded execution limits how far a run can be parallelised. Separate runs can
still execute in parallel, which is what parameter sweeps need.
