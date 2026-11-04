#include <limits>

#include <gtest/gtest.h>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/plant/actuator.hpp"

namespace {

using stapes::plant::friction_torque;
using stapes::plant::FrictionParams;
using stapes::plant::gear_torque;
using stapes::plant::GearParams;
using stapes::plant::saturate_torque;

constexpr FrictionParams kFriction{1.0, 0.5, 0.1, 0.2, 0.001};
constexpr GearParams kGear{1000.0, 10.0, 0.01};

TEST(Friction, IsZeroAtRest) {
  stapes::test::verifies("SRS-ACT-001");
  EXPECT_EQ(friction_torque(kFriction, 0.0), 0.0);
}

TEST(Friction, ReachesCoulombPlusViscousAtHighSpeed) {
  stapes::test::verifies("SRS-ACT-001");
  EXPECT_NEAR(friction_torque(kFriction, 5.0), 1.0 + (0.2 * 5.0), 1e-9);
}

TEST(Friction, IsHighestJustAboveBreakaway) {
  // The Stribeck effect: moving at a crawl takes more force than moving faster.
  stapes::test::verifies("SRS-ACT-001");
  const double crawling = friction_torque(kFriction, 0.01);
  const double moving = friction_torque(kFriction, 0.5);
  EXPECT_NEAR(crawling, 1.5, 0.05);
  EXPECT_GT(crawling, moving);
}

TEST(Friction, AlwaysOpposesMotionAndIsOdd) {
  stapes::test::verifies("SRS-ACT-001");
  stapes::test::for_all_cases(601, 5000, [](stapes::base::Rng& rng) {
    const double velocity = rng.uniform(1e-6, 10.0);
    const double torque = friction_torque(kFriction, velocity);
    EXPECT_GT(torque, 0.0);
    EXPECT_EQ(friction_torque(kFriction, -velocity), -torque);
  });
}

TEST(Backlash, TransmitsNothingInsideTheDeadZone) {
  stapes::test::verifies("SRS-ACT-002");
  stapes::test::for_all_cases(603, 5000, [](stapes::base::Rng& rng) {
    const double deflection = rng.uniform(-0.01, 0.01);
    EXPECT_EQ(gear_torque(kGear, deflection, rng.uniform(-5.0, 5.0)), 0.0);
  });
}

TEST(Backlash, IsLinearInPenetrationBeyondTheDeadZone) {
  stapes::test::verifies("SRS-ACT-002");
  EXPECT_NEAR(gear_torque(kGear, 0.03, 0.0), 1000.0 * 0.02, 1e-9);
  EXPECT_NEAR(gear_torque(kGear, -0.03, 0.0), -1000.0 * 0.02, 1e-9);
}

TEST(Backlash, DampingAddsToTheContactForce) {
  stapes::test::verifies("SRS-ACT-002");
  EXPECT_NEAR(gear_torque(kGear, 0.03, 0.5), (1000.0 * 0.02) + (10.0 * 0.5), 1e-9);
  EXPECT_NEAR(gear_torque(kGear, -0.03, -0.5), -(1000.0 * 0.02) - (10.0 * 0.5), 1e-9);
}

TEST(Backlash, TeethNeverPull) {
  // Separating fast, damping alone would give a negative force, but gear teeth only push.
  stapes::test::verifies("SRS-ACT-002");
  EXPECT_EQ(gear_torque(kGear, 0.011, -100.0), 0.0);
  EXPECT_EQ(gear_torque(kGear, -0.011, 100.0), 0.0);
}

TEST(TorqueLimit, ClampsInBothDirections) {
  stapes::test::verifies("SRS-ACT-003");
  EXPECT_EQ(saturate_torque(100.0, 10.0), 10.0);
  EXPECT_EQ(saturate_torque(-100.0, 10.0), -10.0);
  EXPECT_EQ(saturate_torque(3.0, 10.0), 3.0);
}

TEST(TorqueLimit, TurnsNaNIntoNoTorque) {
  stapes::test::verifies("SRS-ACT-003");
  EXPECT_EQ(saturate_torque(std::numeric_limits<double>::quiet_NaN(), 10.0), 0.0);
}

}  // namespace
