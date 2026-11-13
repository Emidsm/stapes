#pragma once

#include "stapes/base/random.hpp"
#include "stapes/math/rotation.hpp"

namespace stapes::plant {

struct TrackerParams {
  double position_noise_m;       ///< Standard deviation per axis.
  double orientation_noise_rad;  ///< Standard deviation per axis, as a small rotation.
};

/// An optical tracker observing the tool from outside the arm, so it sees where the tool
/// really is, including anything the encoders cannot: gearbox flex, or a seized joint.
///
/// Samples at the control rate with no latency. A real tracker is slower and has both, so
/// this is optimistic; the comms delay fault covers late data.
class OpticalTracker {
 public:
  /// @throws std::invalid_argument if either noise is negative or not finite.
  OpticalTracker(const TrackerParams& params, const base::Rng& rng);

  [[nodiscard]] math::Pose sample(const math::Pose& true_pose) noexcept;

 private:
  TrackerParams params_;
  base::Rng rng_;
};

}  // namespace stapes::plant
