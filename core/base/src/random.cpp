#include "stapes/base/random.hpp"

#include <bit>
#include <cmath>

namespace stapes::base {

namespace {

// SplitMix64 output is a bijection of its counter, so four consecutive outputs are
// never all zero, which is the one state xoshiro cannot use.
Xoshiro256pp::State expand_seed(std::uint64_t seed, std::uint64_t stream) noexcept {
  SplitMix64 seed_mixer(seed);
  SplitMix64 stream_mixer(stream);
  SplitMix64 expander(seed_mixer.next() ^ std::rotl(stream_mixer.next(), 32));
  return {expander.next(), expander.next(), expander.next(), expander.next()};
}

}  // namespace

std::uint64_t SplitMix64::next() noexcept {
  state_ += 0x9E3779B97F4A7C15ULL;
  std::uint64_t z = state_;
  z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31U);
}

std::uint64_t Xoshiro256pp::next() noexcept {
  const std::uint64_t result = std::rotl(state_[0] + state_[3], 23) + state_[0];
  const std::uint64_t shifted = state_[1] << 17U;

  state_[2] ^= state_[0];
  state_[3] ^= state_[1];
  state_[1] ^= state_[2];
  state_[0] ^= state_[3];
  state_[2] ^= shifted;
  state_[3] = std::rotl(state_[3], 45);

  return result;
}

Rng::Rng(std::uint64_t seed, std::uint64_t stream) noexcept : engine_(expand_seed(seed, stream)) {}

double Rng::uniform01() noexcept {
  // The top 53 bits scaled by 2^-53 give every representable step in [0, 1).
  constexpr double kScale = 0x1.0p-53;
  return static_cast<double>(next_u64() >> 11U) * kScale;
}

double Rng::uniform(double lower, double upper) noexcept {
  return lower + ((upper - lower) * uniform01());
}

double Rng::gaussian() noexcept {
  if (has_spare_gaussian_) {
    has_spare_gaussian_ = false;
    return spare_gaussian_;
  }

  double x = 0.0;
  double y = 0.0;
  double radius_squared = 0.0;
  do {
    x = uniform(-1.0, 1.0);
    y = uniform(-1.0, 1.0);
    radius_squared = (x * x) + (y * y);
  } while (radius_squared >= 1.0 || radius_squared == 0.0);

  const double scale = std::sqrt(-2.0 * std::log(radius_squared) / radius_squared);
  spare_gaussian_ = y * scale;
  has_spare_gaussian_ = true;
  return x * scale;
}

}  // namespace stapes::base
