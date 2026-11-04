#pragma once

#include <bit>
#include <cstdint>
#include <span>

namespace stapes::base {

/// FNV-1a over the exact bits of a run's state.
///
/// Two runs have the same hash only if every value fed in matched to the last bit,
/// which is a much stronger check than comparing results within a tolerance. This
/// detects accidental differences; it is not a cryptographic hash.
class StateHash {
 public:
  constexpr void add(std::uint64_t value) noexcept {
    // Byte by byte in a fixed order, so the result does not depend on endianness.
    for (unsigned shift = 0; shift < 64U; shift += 8U) {
      hash_ ^= (value >> shift) & 0xFFU;
      hash_ *= kPrime;
    }
  }

  constexpr void add(double value) noexcept { add(std::bit_cast<std::uint64_t>(value)); }

  constexpr void add(std::span<const double> values) noexcept {
    for (const double value : values) {
      add(value);
    }
  }

  [[nodiscard]] constexpr std::uint64_t value() const noexcept { return hash_; }

 private:
  static constexpr std::uint64_t kOffsetBasis = 0xCBF29CE484222325ULL;
  static constexpr std::uint64_t kPrime = 0x100000001B3ULL;

  std::uint64_t hash_{kOffsetBasis};
};

}  // namespace stapes::base
