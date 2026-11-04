#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/dynamics/dynamic_model.hpp"

namespace {

using stapes::dynamics::DynamicModel;

const DynamicModel kModel = DynamicModel::reference();

TEST(DynamicModel, AcceptsTheReferenceArm) {
  stapes::test::verifies("SRS-DYN-001");
  EXPECT_GT(kModel.links()[0].mass_kg, 0.0);
  EXPECT_TRUE(kModel.gravity().isApprox(Eigen::Vector3d(0.0, 0.0, -9.81)));
}

TEST(DynamicModel, RejectsNonPositiveMass) {
  stapes::test::verifies("SRS-DYN-001");
  DynamicModel::Links links = kModel.links();
  links[1].mass_kg = 0.0;
  EXPECT_THROW(DynamicModel(kModel.arm(), links, kModel.gravity()), std::invalid_argument);
  links[1].mass_kg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(DynamicModel(kModel.arm(), links, kModel.gravity()), std::invalid_argument);
}

TEST(DynamicModel, RejectsNonFiniteCentreOfMass) {
  stapes::test::verifies("SRS-DYN-001");
  DynamicModel::Links links = kModel.links();
  links[2].com_m.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(DynamicModel(kModel.arm(), links, kModel.gravity()), std::invalid_argument);
}

TEST(DynamicModel, RejectsAsymmetricInertia) {
  stapes::test::verifies("SRS-DYN-001");
  DynamicModel::Links links = kModel.links();
  links[0].inertia_kgm2(0, 1) += 0.01;
  EXPECT_THROW(DynamicModel(kModel.arm(), links, kModel.gravity()), std::invalid_argument);
}

TEST(DynamicModel, RejectsNegativeInertia) {
  stapes::test::verifies("SRS-DYN-001");
  DynamicModel::Links links = kModel.links();
  links[0].inertia_kgm2(1, 1) = -0.01;
  EXPECT_THROW(DynamicModel(kModel.arm(), links, kModel.gravity()), std::invalid_argument);
}

TEST(DynamicModel, RejectsInertiaThatBreaksTheTriangleInequality) {
  // Positive-definite, yet no rigid body has these moments: 1 > 0.1 + 0.1.
  stapes::test::verifies("SRS-DYN-001");
  DynamicModel::Links links = kModel.links();
  links[0].inertia_kgm2 = Eigen::Vector3d(0.1, 0.1, 1.0).asDiagonal();
  EXPECT_THROW(DynamicModel(kModel.arm(), links, kModel.gravity()), std::invalid_argument);
}

TEST(DynamicModel, RejectsNonFiniteGravity) {
  stapes::test::verifies("SRS-DYN-001");
  const Eigen::Vector3d gravity(0.0, 0.0, std::numeric_limits<double>::quiet_NaN());
  EXPECT_THROW(DynamicModel(kModel.arm(), kModel.links(), gravity), std::invalid_argument);
}

}  // namespace
