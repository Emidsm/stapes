#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"
#include "stapes/task/keyhole.hpp"

namespace {

using stapes::kinematics::ArmModel;
using stapes::kinematics::forward_kinematics;
using stapes::kinematics::JointVector;
using stapes::kinematics::reference_home;
using stapes::math::Pose;
using stapes::task::axis_distance;
using stapes::task::FeedProfile;
using stapes::task::KeyholeTrajectory;
using stapes::task::plan_is_reachable;
using stapes::task::pose_on_axis;
using stapes::task::pose_through_fulcrum;
using stapes::task::tool_axis;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kTolerance = 1e-12;

Eigen::Vector3d random_vector(stapes::base::Rng& rng, double extent) {
  return {rng.uniform(-extent, extent), rng.uniform(-extent, extent), rng.uniform(-extent, extent)};
}

Pose random_pose(stapes::base::Rng& rng) {
  Pose pose = Pose::Identity();
  pose.translation() = random_vector(rng, 1.0);
  pose.linear() = Eigen::AngleAxisd(rng.uniform(0.0, 3.0), random_vector(rng, 1.0).normalized())
                      .toRotationMatrix();
  return pose;
}

// ---- The constraint --------------------------------------------------------------------------

TEST(ToolAxis, IsTheFirstColumnOfTheToolRotation) {
  stapes::test::verifies("SRS-KEY-001");
  Pose pose = Pose::Identity();
  pose.linear() =
      Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(tool_axis(pose).isApprox(Eigen::Vector3d::UnitY(), kTolerance));
}

TEST(AxisDistance, MatchesHandWorkedCases) {
  stapes::test::verifies("SRS-KEY-001");
  Pose tool = Pose::Identity();
  tool.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  EXPECT_NEAR(axis_distance(tool, {5.0, 0.0, 0.0}), 0.0, kTolerance);   // On the axis, ahead.
  EXPECT_NEAR(axis_distance(tool, {-5.0, 0.0, 0.0}), 0.0, kTolerance);  // On the axis, behind.
  EXPECT_NEAR(axis_distance(tool, {1.0, 3.0, 4.0}), 5.0, kTolerance);   // Level with the tip.
  EXPECT_NEAR(axis_distance(tool, {9.0, 0.0, 2.0}), 2.0, kTolerance);   // Beyond the tip.
}

TEST(PoseOnAxis, PutsTheTipAndAxisWhereAsked) {
  stapes::test::verifies("SRS-KEY-002");
  stapes::test::for_all_cases(11, 200, [](stapes::base::Rng& rng) {
    const Pose reference = random_pose(rng);
    const Eigen::Vector3d tip = random_vector(rng, 1.0);
    const Eigen::Vector3d axis = random_vector(rng, 1.0).normalized();
    const Pose pose = pose_on_axis(tip, axis, reference);
    EXPECT_TRUE(pose.translation().isApprox(tip, kTolerance));
    EXPECT_TRUE(tool_axis(pose).isApprox(axis, 1e-9));
    EXPECT_TRUE(pose.linear().isUnitary(1e-9));
    EXPECT_GT(pose.linear().determinant(), 0.0);
  });
}

TEST(PoseOnAxis, TurnsByNoMoreThanTheAngleBetweenTheAxes) {
  // The smallest rotation adds no roll about the tool axis. Any other choice would.
  stapes::test::verifies("SRS-KEY-002");
  stapes::test::for_all_cases(12, 200, [](stapes::base::Rng& rng) {
    const Pose reference = random_pose(rng);
    const Eigen::Vector3d axis = random_vector(rng, 1.0).normalized();
    const Pose pose = pose_on_axis(Eigen::Vector3d::Zero(), axis, reference);
    const double between = std::acos(std::clamp(tool_axis(reference).dot(axis), -1.0, 1.0));
    const double turned = Eigen::AngleAxisd(pose.linear() * reference.linear().transpose()).angle();
    EXPECT_NEAR(turned, between, 1e-7);
  });
}

TEST(PoseOnAxis, KeepsTheOrientationWhenTheAxisAlreadyMatches) {
  stapes::test::verifies("SRS-KEY-002");
  const Pose reference = forward_kinematics(ArmModel::reference(), reference_home());
  const Pose pose = pose_on_axis(Eigen::Vector3d::Zero(), tool_axis(reference), reference);
  EXPECT_TRUE(pose.linear().isApprox(reference.linear(), 1e-12));
}

TEST(PoseThroughFulcrum, TheAxisPassesThroughTheFulcrumPointingAwayFromIt) {
  stapes::test::verifies("SRS-KEY-002");
  stapes::test::for_all_cases(13, 200, [](stapes::base::Rng& rng) {
    const Pose reference = random_pose(rng);
    const Eigen::Vector3d tip = random_vector(rng, 1.0);
    const Eigen::Vector3d fulcrum = tip + random_vector(rng, 0.5) + Eigen::Vector3d(0.6, 0.0, 0.0);
    const Pose pose = pose_through_fulcrum(tip, fulcrum, reference);
    EXPECT_NEAR(axis_distance(pose, fulcrum), 0.0, 1e-9);
    EXPECT_TRUE(pose.translation().isApprox(tip, kTolerance));
    EXPECT_GT(tool_axis(pose).dot(tip - fulcrum), 0.0);
  });
}

// ---- The tunnel ------------------------------------------------------------------------------

TEST(KeyholeTrajectory, GeometryMatchesHandWorkedValues) {
  stapes::test::verifies("SRS-KEY-003");
  const KeyholeTrajectory tunnel({1.0, 0.0, 0.0}, {1.0, 0.0, 2.0}, Pose::Identity());
  EXPECT_DOUBLE_EQ(tunnel.length_m(), 2.0);
  EXPECT_TRUE(tunnel.direction().isApprox(Eigen::Vector3d::UnitZ(), kTolerance));
  EXPECT_NEAR(tunnel.depth_of({1.0, 7.0, 0.5}), 0.5, kTolerance);
  EXPECT_NEAR(tunnel.depth_of({1.0, 0.0, -1.0}), -1.0, kTolerance);
  EXPECT_NEAR(tunnel.depth_of({1.0, 0.0, 3.0}), 3.0, kTolerance);
  EXPECT_NEAR(tunnel.radial_distance({4.0, 4.0, 1.0}), 5.0, kTolerance);
  EXPECT_NEAR(tunnel.radial_distance({1.0, 0.0, 99.0}), 0.0, kTolerance);
  EXPECT_TRUE(tunnel.entry().isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_TRUE(tunnel.target().isApprox(Eigen::Vector3d(1.0, 0.0, 2.0)));
}

TEST(KeyholeTrajectory, PosesLieOnTheLineAndPointAlongIt) {
  stapes::test::verifies("SRS-KEY-003");
  const KeyholeTrajectory tunnel({1.0, 0.0, 0.0}, {1.0, 0.0, 2.0}, Pose::Identity());
  for (const double depth : {0.0, 0.5, 2.0}) {
    const Pose pose = tunnel.pose_at(depth);
    EXPECT_NEAR(tunnel.depth_of(pose.translation()), depth, kTolerance);
    EXPECT_NEAR(tunnel.radial_distance(pose.translation()), 0.0, kTolerance);
    EXPECT_TRUE(tool_axis(pose).isApprox(tunnel.direction(), 1e-9));
    EXPECT_NEAR(axis_distance(pose, tunnel.entry()), 0.0, 1e-9);
  }
}

TEST(KeyholeTrajectory, DepthsAreClampedToTheTunnel) {
  stapes::test::verifies("SRS-KEY-003");
  const KeyholeTrajectory tunnel({0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, Pose::Identity());
  EXPECT_TRUE(tunnel.pose_at(-1.0).translation().isApprox(tunnel.entry(), kTolerance));
  EXPECT_TRUE(tunnel.pose_at(99.0).translation().isApprox(tunnel.target(), kTolerance));
}

TEST(KeyholeTrajectory, RejectsPointsThatDoNotDefineATunnel) {
  stapes::test::verifies("SRS-KEY-003");
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  EXPECT_THROW(KeyholeTrajectory(origin, origin, Pose::Identity()), std::invalid_argument);
  EXPECT_THROW(
      KeyholeTrajectory(Eigen::Vector3d(kNaN, 0.0, 0.0), {1.0, 0.0, 0.0}, Pose::Identity()),
      std::invalid_argument);
  EXPECT_THROW(KeyholeTrajectory(origin, Eigen::Vector3d(0.0, kNaN, 0.0), Pose::Identity()),
               std::invalid_argument);
}

// ---- The feed --------------------------------------------------------------------------------

TEST(FeedProfile, MatchesHandWorkedTrapezoid) {
  // 10 mm at 5 mm/s with 25 mm/s^2: ramps take 0.2 s and cover 0.5 mm each, and 9 mm of cruising
  // takes 1.8 s, so the whole move takes 2.2 s.
  stapes::test::verifies("SRS-KEY-004");
  const FeedProfile feed(10e-3, 5e-3, 25e-3);
  EXPECT_NEAR(feed.duration_s(), 2.2, 1e-12);
  EXPECT_NEAR(feed.depth_at(0.1), 0.5 * 25e-3 * 0.01, 1e-12);
  EXPECT_NEAR(feed.depth_at(0.2), 0.5e-3, 1e-12);
  EXPECT_NEAR(feed.depth_at(1.1), 5e-3, 1e-12);
  EXPECT_NEAR(feed.depth_at(2.0), 9.5e-3, 1e-12);
  EXPECT_NEAR(feed.depth_at(2.1), 10e-3 - (0.5 * 25e-3 * 0.01), 1e-12);
}

TEST(FeedProfile, ClampsOutsideTheMove) {
  stapes::test::verifies("SRS-KEY-004");
  const FeedProfile feed(10e-3, 5e-3, 25e-3);
  EXPECT_EQ(feed.depth_at(-1.0), 0.0);
  EXPECT_EQ(feed.depth_at(0.0), 0.0);
  EXPECT_EQ(feed.depth_at(feed.duration_s()), 10e-3);
  EXPECT_EQ(feed.depth_at(100.0), 10e-3);
}

TEST(FeedProfile, AShortMoveBecomesATriangle) {
  // 1 mm at 25 mm/s^2 peaks at sqrt(1e-3 * 25e-3) = 5 mm/s, so a 100 mm/s limit is never reached.
  stapes::test::verifies("SRS-KEY-004");
  const FeedProfile feed(1e-3, 100e-3, 25e-3);
  EXPECT_NEAR(feed.duration_s(), 0.4, 1e-12);
  EXPECT_NEAR(feed.depth_at(0.2), 0.5e-3, 1e-12);
}

TEST(FeedProfile, IsContinuousMonotonicAndRespectsSpeedAndAcceleration) {
  stapes::test::verifies("SRS-KEY-004");
  stapes::test::for_all_cases(14, 100, [](stapes::base::Rng& rng) {
    const double length = rng.uniform(1e-3, 50e-3);
    const double speed = rng.uniform(0.5e-3, 20e-3);
    const double acceleration = rng.uniform(5e-3, 100e-3);
    const FeedProfile feed(length, speed, acceleration);
    constexpr double kStep = 1e-3;
    double previous_depth = 0.0;
    double previous_speed = 0.0;
    const int steps = static_cast<int>(std::ceil(feed.duration_s() / kStep)) + 1;
    for (int step = 1; step <= steps; ++step) {
      const double depth = feed.depth_at(step * kStep);
      const double current_speed = (depth - previous_depth) / kStep;
      EXPECT_GE(depth, previous_depth);
      EXPECT_LE(current_speed, speed * (1.0 + 1e-9));
      EXPECT_LE(std::abs(current_speed - previous_speed), (acceleration * kStep * 1.5) + 1e-9);
      previous_depth = depth;
      previous_speed = current_speed;
    }
    EXPECT_NEAR(previous_depth, length, 1e-15);
  });
}

TEST(FeedProfile, RejectsNonPositiveValues) {
  stapes::test::verifies("SRS-KEY-004");
  EXPECT_THROW(FeedProfile(0.0, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(FeedProfile(1.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(FeedProfile(1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(FeedProfile(kNaN, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(FeedProfile(1.0, -1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(FeedProfile(1.0, 1.0, std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

// ---- Checking the plan -----------------------------------------------------------------------

TEST(PlanIsReachable, AcceptsAReachableTunnelAndFindsItsStart) {
  stapes::test::verifies("SRS-KEY-005");
  const ArmModel model = ArmModel::reference();
  const Pose start = forward_kinematics(model, reference_home());
  const Eigen::Vector3d target = start.translation() + (10e-3 * tool_axis(start));
  const KeyholeTrajectory tunnel(start.translation(), target, start);
  JointVector found = JointVector::Zero();
  EXPECT_TRUE(plan_is_reachable(model, tunnel, reference_home(), 1e-3, found));
  EXPECT_TRUE(forward_kinematics(model, found).translation().isApprox(start.translation(), 1e-5));
}

TEST(PlanIsReachable, RefusesATunnelThatStartsOutOfReach) {
  stapes::test::verifies("SRS-KEY-005");
  const ArmModel model = ArmModel::reference();
  const KeyholeTrajectory tunnel({5.0, 0.0, 0.0}, {5.1, 0.0, 0.0}, Pose::Identity());
  JointVector found = JointVector::Zero();
  EXPECT_FALSE(plan_is_reachable(model, tunnel, reference_home(), 1e-3, found));
}

TEST(PlanIsReachable, RefusesAnUnreachableEntryEvenWhenTheTargetIsInReach) {
  // One step covers the whole tunnel, so the target is the only other point solved. It passes,
  // which leaves the check on the entry as the only thing that can refuse this plan.
  stapes::test::verifies("SRS-KEY-005");
  const ArmModel model = ArmModel::reference();
  const Pose inside = forward_kinematics(model, reference_home());
  const Eigen::Vector3d entry = inside.translation() - (3.0 * tool_axis(inside));
  const KeyholeTrajectory tunnel(entry, inside.translation(), inside);
  ASSERT_TRUE(
      stapes::kinematics::solve_ik(model, tunnel.pose_at(tunnel.length_m()), reference_home())
          .converged());
  JointVector found = JointVector::Zero();
  EXPECT_FALSE(plan_is_reachable(model, tunnel, reference_home(), 10.0, found));
}

TEST(PlanIsReachable, RefusesATunnelThatLeavesTheWorkspacePartWay) {
  // Starts within reach, so only walking the whole length finds the problem.
  stapes::test::verifies("SRS-KEY-005");
  const ArmModel model = ArmModel::reference();
  const Pose start = forward_kinematics(model, reference_home());
  const Eigen::Vector3d target = start.translation() + (3.0 * tool_axis(start));
  const KeyholeTrajectory tunnel(start.translation(), target, start);
  JointVector found = JointVector::Zero();
  EXPECT_FALSE(plan_is_reachable(model, tunnel, reference_home(), 10e-3, found));
}

TEST(PlanIsReachable, RefusesAStepThatCouldNeverFinish) {
  stapes::test::verifies("SRS-KEY-005");
  const ArmModel model = ArmModel::reference();
  const Pose start = forward_kinematics(model, reference_home());
  const KeyholeTrajectory tunnel(start.translation(),
                                 start.translation() + (10e-3 * tool_axis(start)), start);
  JointVector found = JointVector::Zero();
  EXPECT_FALSE(plan_is_reachable(model, tunnel, reference_home(), 0.0, found));
  EXPECT_FALSE(plan_is_reachable(model, tunnel, reference_home(), -1.0, found));
  EXPECT_FALSE(plan_is_reachable(model, tunnel, reference_home(), kNaN, found));
}

}  // namespace
