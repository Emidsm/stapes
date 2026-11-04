#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/kinematics/arm_model.hpp"

namespace {

using stapes::kinematics::ArmModel;
using stapes::kinematics::JointVector;
using stapes::kinematics::reference_home;

const ArmModel kModel = ArmModel::reference();

TEST(ArmModel, ReferenceHomeIsWithinLimits) {
  stapes::test::verifies("SRS-KIN-006");
  EXPECT_TRUE(kModel.within_limits(reference_home()));
}

TEST(ArmModel, RejectsNonUnitAxis) {
  stapes::test::verifies("SRS-KIN-006");
  ArmModel::Joints joints = kModel.joints();
  joints[2].axis = Eigen::Vector3d(0.0, 2.0, 0.0);
  EXPECT_THROW(ArmModel(joints, kModel.tool()), std::invalid_argument);
}

TEST(ArmModel, RejectsEmptyLimitRange) {
  stapes::test::verifies("SRS-KIN-006");
  ArmModel::Joints joints = kModel.joints();
  joints[0].lower_rad = joints[0].upper_rad;
  EXPECT_THROW(ArmModel(joints, kModel.tool()), std::invalid_argument);
}

TEST(ArmModel, RejectsNonFiniteLimit) {
  stapes::test::verifies("SRS-KIN-006");
  ArmModel::Joints joints = kModel.joints();
  joints[0].upper_rad = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ArmModel(joints, kModel.tool()), std::invalid_argument);
}

TEST(ArmModel, RejectsNonRigidJointOrigin) {
  stapes::test::verifies("SRS-KIN-006");
  ArmModel::Joints joints = kModel.joints();
  joints[3].origin.linear() *= 2.0;
  EXPECT_THROW(ArmModel(joints, kModel.tool()), std::invalid_argument);
}

TEST(ArmModel, RejectsReflectionAsTool) {
  stapes::test::verifies("SRS-KIN-006");
  stapes::math::Pose mirrored = stapes::math::Pose::Identity();
  mirrored.linear()(0, 0) = -1.0;
  EXPECT_THROW(ArmModel(kModel.joints(), mirrored), std::invalid_argument);
}

TEST(ArmModel, ClampingAlwaysLandsWithinLimits) {
  stapes::test::verifies("SRS-KIN-006");
  stapes::test::for_all_cases(201, 2000, [](stapes::base::Rng& rng) {
    JointVector q;
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      q(i) = rng.uniform(-10.0, 10.0);
    }
    EXPECT_TRUE(kModel.within_limits(kModel.clamp_to_limits(q)));
  });
}

TEST(ArmModel, ClampingLeavesValidConfigurationsAlone) {
  stapes::test::verifies("SRS-KIN-006");
  stapes::test::for_all_cases(203, 2000, [](stapes::base::Rng& rng) {
    const JointVector q = stapes::test::random_configuration(kModel, rng);
    EXPECT_EQ(kModel.clamp_to_limits(q), q);
  });
}

TEST(ArmModel, NaNIsNeverWithinLimits) {
  stapes::test::verifies("SRS-KIN-006");
  JointVector q = reference_home();
  q(4) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kModel.within_limits(q));
}

}  // namespace
