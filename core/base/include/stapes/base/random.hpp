#pragma once

#include <array>
#include <cstdint>

namespace stapes::base {

/// SplitMix64 (Steele, Lea, Flood, 2014). Expands one 64-bit seed into a state.
class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) noexcept : state_(seed) {}

  [[nodiscard]] std::uint64_t next() noexcept;

 private:
  std::uint64_t state_;
};

/// xoshiro256++ 1.0 (Blackman, Vigna, 2019).
class Xoshiro256pp {
 public:
  using State = std::array<std::uint64_t, 4>;

  /// `state` must not be all zero, the one state the generator cannot leave.
  explicit Xoshiro256pp(const State& state) noexcept : state_(state) {}

  [[nodiscard]] std::uint64_t next() noexcept;

 private:
  State state_;
};

/// The only source of randomness in the simulation.
///
/// `std::mt19937_64` output is fixed by the standard, but `std::normal_distribution`
/// is not: libstdc++ and libc++ return different values for the same engine state.
/// Writing the sampler here makes the sequence part of this code base.
///
/// Each noise source owns a stream. Streams are independent objects, so adding a
/// sensor to a scenario does not change the noise that the existing sensors see.
///
/// `uniform01()` is exact for a given seed everywhere. `gaussian()` calls
/// `std::log`, whose rounding belongs to the platform's libm, so it is only
/// bit-exact for a given libm.
class Rng {
 public:
  explicit Rng(std::uint64_t seed, std::uint64_t stream = 0) noexcept;

  [[nodiscard]] std::uint64_t next_u64() noexcept { return engine_.next(); }

  /// Uniform in [0, 1).
  [[nodiscard]] double uniform01() noexcept;

  /// Uniform in [lower, upper).
  [[nodiscard]] double uniform(double lower, double upper) noexcept;

  /// Standard normal, by the Marsaglia polar method.
  [[nodiscard]] double gaussian() noexcept;

  [[nodiscard]] double gaussian(double mean, double stddev) noexcept {
    return mean + (stddev * gaussian());
  }

 private:
  Xoshiro256pp engine_;
  double spare_gaussian_{0.0};
  bool has_spare_gaussian_{false};
};

}  // namespace stapes::base
