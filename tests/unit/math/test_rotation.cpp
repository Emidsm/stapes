#include <cmath>
#include <numbers>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/math/rotation.hpp"

namespace {

using stapes::math::orientation_error;
using stapes::math::rotation_vector;

// Braces, not parentheses: a braced list is evaluated left to right, so the draws reach the
// same axes with every compiler.
Eigen::Vector3d random_unit_vector(stapes::base::Rng& rng) {
  return Eigen::Vector3d{rng.gaussian(), rng.gaussian(), rng.gaussian()}.normalized();
}

TEST(RotationVector, IsZeroForIdentity) {
  stapes::test::verifies("SRS-KIN-007");
  EXPECT_TRUE(rotation_vector(Eigen::Matrix3d::Identity()).isZero(1e-15));
}

TEST(RotationVector, RecoversAxisAndAngle) {
  stapes::test::verifies("SRS-KIN-007");
  stapes::test::for_all_cases(101, 2000, [](stapes::base::Rng& rng) {
    const Eigen::Vector3d axis = random_unit_vector(rng);
    // Stops short of pi, where the axis direction flips sign.
    const double angle = rng.uniform(0.0, std::numbers::pi - 1e-6);
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    EXPECT_TRUE(rotation_vector(rotation).isApprox(angle * axis, 1e-9));
  });
}

TEST(RotationVector, IsAccurateForVerySmallAngles) {
  stapes::test::verifies("SRS-KIN-007");
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
  for (const double angle : {1e-4, 1e-7, 1e-9, 1e-12}) {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    EXPECT_TRUE((rotation_vector(rotation) - (angle * axis)).isZero(1e-14)) << angle;
  }
}

TEST(RotationVector, HasTheRightMagnitudeAtHalfTurn) {
  stapes::test::verifies("SRS-KIN-007");
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 1.0, 0.0).normalized();
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(std::numbers::pi, axis).toRotationMatrix();
  EXPECT_NEAR(rotation_vector(rotation).norm(), std::numbers::pi, 1e-9);
}

TEST(OrientationError, ApplyingTheErrorReachesTheTarget) {
  stapes::test::verifies("SRS-KIN-007");
  stapes::test::for_all_cases(103, 2000, [](stapes::base::Rng& rng) {
    const Eigen::Matrix3d actual =
        Eigen::AngleAxisd(rng.uniform(0.0, 3.0), random_unit_vector(rng)).toRotationMatrix();
    const Eigen::Matrix3d target =
        Eigen::AngleAxisd(rng.uniform(0.0, 3.0), random_unit_vector(rng)).toRotationMatrix();

    const Eigen::Vector3d error = orientation_error(target, actual);
    const Eigen::Matrix3d corrected =
        Eigen::AngleAxisd(error.norm(), error.normalized()).toRotationMatrix() * actual;
    EXPECT_TRUE(corrected.isApprox(target, 1e-9));
  });
}

}  // namespace
