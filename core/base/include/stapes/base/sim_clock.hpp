#pragma once

#include <chrono>
#include <cstdint>

namespace stapes::base {

using SimDuration = std::chrono::nanoseconds;

/// Simulated time, stored as an integer number of fixed steps.
///
/// Time is never accumulated in floating point: adding 0.001 s a million times
/// does not give 1000 s, and that error would make long runs drift. Nothing in the
/// simulation reads the wall clock.
class SimClock {
 public:
  /// @throws std::invalid_argument if `step` is not positive.
  explicit SimClock(SimDuration step);

  void advance() noexcept { ++ticks_; }

  [[nodiscard]] std::int64_t ticks() const noexcept { return ticks_; }
  [[nodiscard]] SimDuration step() const noexcept { return step_; }
  [[nodiscard]] SimDuration now() const noexcept { return step_ * ticks_; }
  [[nodiscard]] double now_seconds() const noexcept {
    return std::chrono::duration<double>(now()).count();
  }

 private:
  SimDuration step_;
  std::int64_t ticks_{0};
};

}  // namespace stapes::base
