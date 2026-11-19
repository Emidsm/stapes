#include "stapes/task/joint_move.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace stapes::task {

namespace {

// The peak of the quintic's derivative, 30s^2 - 60s^3 + 30s^4, reached at s = 1/2.
constexpr double kPeakSpeedFactor = 1.875;

}  // namespace

JointMove::JointMove(const kinematics::JointVector& from, const kinematics::JointVector& to,
                     double duration_s)
    : from_(from), to_(to), duration_s_(duration_s) {
  if (!from.allFinite() || !to.allFinite() || !std::isfinite(duration_s) || !(duration_s > 0.0)) {
    throw std::invalid_argument("a joint move needs finite ends and a positive duration");
  }
}

kinematics::JointVector JointMove::at(double time_s) const noexcept {
  const double s = std::clamp(time_s / duration_s_, 0.0, 1.0);
  const double blend = s * s * s * (10.0 + (s * (-15.0 + (6.0 * s))));
  // Weighted this way rather than `from + blend * (to - from)`, which does not give `to` exactly
  // when the blend is 1. The move must end exactly where the feed starts.
  return ((1.0 - blend) * from_) + (blend * to_);
}

kinematics::JointVector JointMove::peak_speeds_rad_s() const noexcept {
  return (kPeakSpeedFactor / duration_s_) * (to_ - from_).cwiseAbs();
}

}  // namespace stapes::task
