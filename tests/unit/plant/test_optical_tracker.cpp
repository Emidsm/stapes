#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/plant/optical_tracker.hpp"

namespace {

using stapes::base::Rng;
using stapes::math::Pose;
using stapes::plant::OpticalTracker;
using stapes::plant::TrackerParams;

Pose some_pose() {
  Pose pose = Pose::Identity();
  pose.translation() = Eigen::Vector3d(0.3, -0.2, 0.5);
  pose.linear() =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
  return pose;
}

TEST(OpticalTracker, WithoutNoiseItReportsTheTruePose) {
  stapes::test::verifies("SRS-SEN-006");
  OpticalTracker tracker(TrackerParams{0.0, 0.0}, Rng(1));
  EXPECT_TRUE(tracker.sample(some_pose()).matrix().isApprox(some_pose().matrix(), 0.0));
}

TEST(OpticalTracker, PositionNoiseHasTheConfiguredSpreadOnEachAxis) {
  stapes::test::verifies("SRS-SEN-006");
  constexpr double kNoise = 30e-6;
  constexpr int kSamples = 40000;
  OpticalTracker tracker(TrackerParams{kNoise, 0.0}, Rng(2));
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d squares = Eigen::Vector3d::Zero();
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Vector3d error =
        tracker.sample(some_pose()).translation() - some_pose().translation();
    sum += error;
    squares += error.cwiseAbs2();
  }
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(sum(axis) / kSamples, 0.0, 5.0 * kNoise / std::sqrt(kSamples));
    EXPECT_NEAR(std::sqrt(squares(axis) / kSamples), kNoise, 0.03 * kNoise);
  }
}

TEST(OpticalTracker, OrientationNoiseKeepsARotationAndHasTheConfiguredSpread) {
  stapes::test::verifies("SRS-SEN-006");
  constexpr double kNoise = 100e-6;
  constexpr int kSamples = 40000;
  OpticalTracker tracker(TrackerParams{0.0, kNoise}, Rng(3));
  double squares = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const Pose measured = tracker.sample(some_pose());
    ASSERT_TRUE(measured.linear().isUnitary(1e-12));
    ASSERT_GT(measured.linear().determinant(), 0.0);
    const double angle =
        Eigen::AngleAxisd(some_pose().linear().transpose() * measured.linear()).angle();
    squares += angle * angle;
  }
  // Three independent axes, so the squared angle averages three variances.
  EXPECT_NEAR(std::sqrt(squares / kSamples), std::sqrt(3.0) * kNoise, 0.03 * kNoise);
}

TEST(OpticalTracker, TheSameSeedGivesTheSameReadings) {
  stapes::test::verifies("SRS-SEN-006");
  OpticalTracker first(TrackerParams{1e-3, 1e-3}, Rng(4));
  OpticalTracker second(TrackerParams{1e-3, 1e-3}, Rng(4));
  OpticalTracker other(TrackerParams{1e-3, 1e-3}, Rng(5));
  const Pose one = first.sample(some_pose());
  EXPECT_EQ(one.matrix(), second.sample(some_pose()).matrix());
  EXPECT_NE(one.matrix(), other.sample(some_pose()).matrix());
}

TEST(OpticalTracker, RejectsInvalidNoise) {
  stapes::test::verifies("SRS-SEN-006");
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (const TrackerParams bad :
       {TrackerParams{-1.0, 0.0}, TrackerParams{0.0, -1.0}, TrackerParams{kNaN, 0.0},
        TrackerParams{0.0, kNaN}, TrackerParams{kInf, 0.0}, TrackerParams{0.0, kInf}}) {
    EXPECT_THROW(OpticalTracker(bad, Rng(6)), std::invalid_argument);
  }
}

}  // namespace
