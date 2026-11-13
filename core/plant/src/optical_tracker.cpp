#include "stapes/plant/optical_tracker.hpp"

#include <cmath>
#include <stdexcept>

namespace stapes::plant {

OpticalTracker::OpticalTracker(const TrackerParams& params, const base::Rng& rng)
    : params_(params), rng_(rng) {
  if (!std::isfinite(params.position_noise_m) || params.position_noise_m < 0.0 ||
      !std::isfinite(params.orientation_noise_rad) || params.orientation_noise_rad < 0.0) {
    throw std::invalid_argument("tracker noise must be finite and non-negative");
  }
}

math::Pose OpticalTracker::sample(const math::Pose& true_pose) noexcept {
  math::Pose measured = true_pose;
  for (int axis = 0; axis < 3; ++axis) {
    measured.translation()(axis) += params_.position_noise_m * rng_.gaussian();
  }

  // Every draw is made even when the noise is zero, so the number of draws never depends on
  // the parameters.
  const Eigen::Vector3d tilt(params_.orientation_noise_rad * rng_.gaussian(),
                             params_.orientation_noise_rad * rng_.gaussian(),
                             params_.orientation_noise_rad * rng_.gaussian());
  const double angle = tilt.norm();
  if (angle > 0.0) {
    measured.linear() = true_pose.linear() * Eigen::AngleAxisd(angle, tilt / angle);
  }
  return measured;
}

}  // namespace stapes::plant
