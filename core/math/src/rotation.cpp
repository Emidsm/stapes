#include "stapes/math/rotation.hpp"

#include <cmath>

namespace stapes::math {

Eigen::Vector3d rotation_vector(const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();

  // q and -q are the same rotation; the one with w >= 0 has the angle in [0, pi].
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }

  const double half_sine = quaternion.vec().norm();
  // Below this, angle / sin(angle / 2) has reached its limit of 2 to double precision,
  // and dividing by half_sine would only amplify rounding noise.
  constexpr double kSmallAngleHalfSine = 1e-8;
  if (half_sine < kSmallAngleHalfSine) {
    return 2.0 * quaternion.vec();
  }

  const double angle = 2.0 * std::atan2(half_sine, quaternion.w());
  return quaternion.vec() * (angle / half_sine);
}

Eigen::Vector3d orientation_error(const Eigen::Matrix3d& target, const Eigen::Matrix3d& actual) {
  return rotation_vector(target * actual.transpose());
}

}  // namespace stapes::math
