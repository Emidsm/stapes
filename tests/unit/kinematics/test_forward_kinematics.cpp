#include <numbers>

#include <gtest/gtest.h>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"

namespace {

using stapes::kinematics::ArmModel;
using stapes::kinematics::chain_frames;
using stapes::kinematics::forward_kinematics;
using stapes::kinematics::geometric_jacobian;
using stapes::kinematics::Jacobian;
using stapes::kinematics::JointVector;
using stapes::kinematics::manipulability;

constexpr double kQuarterTurn = std::numbers::pi / 2.0;

const ArmModel kModel = ArmModel::reference();

struct KnownPose {
  const char* description;
  std::array<double, 6> q;
  Eigen::Vector3d tip;
};

// Worked out by hand from the reference arm's dimensions, independently of the code.
// At zero: the shoulder is at z = 0.30, the elbow at z = 0.60, and the forearm and tool
// extend 0.45 along +x.
TEST(ForwardKinematics, MatchesPositionsWorkedOutByHand) {
  stapes::test::verifies("SRS-KIN-001");
  const std::array<KnownPose, 7> cases{{
      {"zero", {0, 0, 0, 0, 0, 0}, {0.45, 0.0, 0.60}},
      {"base +90", {kQuarterTurn, 0, 0, 0, 0, 0}, {0.0, 0.45, 0.60}},
      {"shoulder +90", {0, kQuarterTurn, 0, 0, 0, 0}, {0.30, 0.0, -0.15}},
      {"elbow +90", {0, 0, kQuarterTurn, 0, 0, 0}, {0.0, 0.0, 0.15}},
      {"forearm roll +90", {0, 0, 0, kQuarterTurn, 0, 0}, {0.45, 0.0, 0.60}},
      {"wrist +90", {0, 0, 0, 0, kQuarterTurn, 0}, {0.30, 0.0, 0.45}},
      {"tool roll +90", {0, 0, 0, 0, 0, kQuarterTurn}, {0.45, 0.0, 0.60}},
  }};

  for (const KnownPose& known : cases) {
    SCOPED_TRACE(known.description);
    const JointVector q = Eigen::Map<const JointVector>(known.q.data());
    EXPECT_TRUE(forward_kinematics(kModel, q).translation().isApprox(known.tip, 1e-12));
  }
}

TEST(ForwardKinematics, MatchesOrientationsWorkedOutByHand) {
  stapes::test::verifies("SRS-KIN-001");
  JointVector q = JointVector::Zero();
  EXPECT_TRUE(forward_kinematics(kModel, q).linear().isIdentity(1e-12));

  // The base turns the tool about +z by 90 degrees: tool x points along world y.
  q(0) = kQuarterTurn;
  const Eigen::Vector3d tool_x = forward_kinematics(kModel, q).linear().col(0);
  EXPECT_TRUE(tool_x.isApprox(Eigen::Vector3d::UnitY(), 1e-12));
}

TEST(ForwardKinematics, AlwaysProducesARigidTransform) {
  stapes::test::verifies("SRS-KIN-001");
  stapes::test::for_all_cases(301, 2000, [](stapes::base::Rng& rng) {
    const JointVector q = stapes::test::random_configuration(kModel, rng);
    const Eigen::Matrix3d rotation = forward_kinematics(kModel, q).linear();
    EXPECT_TRUE(rotation.isUnitary(1e-12));
    EXPECT_NEAR(rotation.determinant(), 1.0, 1e-12);
  });
}

TEST(ChainFrames, MatchesForwardKinematicsAtTheTip) {
  stapes::test::verifies("SRS-KIN-001");
  const JointVector q = stapes::kinematics::reference_home();
  EXPECT_TRUE(chain_frames(kModel, q).back().isApprox(forward_kinematics(kModel, q), 1e-15));
}

TEST(GeometricJacobian, MatchesFiniteDifferences) {
  stapes::test::verifies("SRS-KIN-002");
  stapes::test::for_all_cases(303, 2000, [](stapes::base::Rng& rng) {
    constexpr double kStep = 1e-6;
    const JointVector q = stapes::test::random_configuration(kModel, rng, 2.0 * kStep);
    const Jacobian analytic = geometric_jacobian(kModel, q);

    for (Eigen::Index joint = 0; joint < q.size(); ++joint) {
      JointVector above = q;
      JointVector below = q;
      above(joint) += kStep;
      below(joint) -= kStep;
      const stapes::math::Pose pose_above = forward_kinematics(kModel, above);
      const stapes::math::Pose pose_below = forward_kinematics(kModel, below);

      const Eigen::Vector3d linear =
          (pose_above.translation() - pose_below.translation()) / (2.0 * kStep);
      const Eigen::Vector3d angular =
          stapes::math::orientation_error(pose_above.linear(), pose_below.linear()) / (2.0 * kStep);

      EXPECT_TRUE((analytic.col(joint).head<3>() - linear).isZero(1e-7)) << "joint " << joint;
      EXPECT_TRUE((analytic.col(joint).tail<3>() - angular).isZero(1e-7)) << "joint " << joint;
    }
  });
}

TEST(Manipulability, VanishesAtTheWristSingularity) {
  stapes::test::verifies("SRS-KIN-008");
  // With the wrist straight, joints 4 and 6 share an axis.
  JointVector q = stapes::kinematics::reference_home();
  q(4) = 0.0;
  EXPECT_NEAR(manipulability(kModel, q), 0.0, 1e-12);
}

TEST(Manipulability, IsPositiveAwayFromSingularities) {
  stapes::test::verifies("SRS-KIN-008");
  EXPECT_GT(manipulability(kModel, stapes::kinematics::reference_home()), 1e-3);
}

}  // namespace
