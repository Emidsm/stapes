#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/base/state_hash.hpp"
#include "stapes/dynamics/rigid_body_dynamics.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/plant/simulated_arm.hpp"

namespace {

using stapes::kinematics::JointVector;
using stapes::plant::SimulatedArm;
using stapes::plant::SimulatedArmConfig;

// ---- Starting state --------------------------------------------------------------------------

TEST(SimulatedArmStart, TheGearboxesAlreadyCarryGravity) {
  // Motors holding the torque gravity needs, from the first moment: the links should not move.
  // If the gearboxes started unloaded, the arm would drop by hundreds of micrometres at the tip.
  stapes::test::verifies("SRS-ARM-007");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(1);
  SimulatedArm arm(config);
  const JointVector gravity = stapes::dynamics::gravity_torques(config.model, config.initial_q);

  double worst = 0.0;
  for (int i = 0; i < 200; ++i) {
    arm.command_torques(gravity);
    arm.tick();
    worst = std::max(worst, (arm.state().q - config.initial_q).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(worst, 2e-6);
}

TEST(SimulatedArmStart, TheLinksStartExactlyWhereConfigured) {
  stapes::test::verifies("SRS-ARM-007");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(1);
  const SimulatedArm arm(config);
  EXPECT_EQ(arm.state().q, config.initial_q);
  EXPECT_TRUE(arm.state().qd.isZero());
  EXPECT_TRUE(arm.state().motor_velocity.isZero());
  EXPECT_NE(arm.state().motor_angle, config.initial_q);
}

TEST(SimulatedArmStart, AJointWithNoLoadHasNoOffset) {
  // In this pose the base yaw axis is vertical, so gravity puts no torque on it.
  stapes::test::verifies("SRS-ARM-007");
  const SimulatedArm arm(SimulatedArmConfig::reference(1));
  EXPECT_DOUBLE_EQ(arm.state().motor_angle(0), arm.state().q(0));
}

// ---- The safety channel's sensors ------------------------------------------------------------

TEST(SafetyChannel, LinkEncodersReadTheLinkNotTheMotor) {
  stapes::test::verifies("SRS-ARM-008");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(2);
  SimulatedArm arm(config);
  arm.tick();
  const JointVector link = arm.read_safety_sensors().link_positions_rad;
  const JointVector motor = arm.read_sensors().joint_positions_rad;
  constexpr double kFiveSigma = 40e-6;
  EXPECT_LT((link - arm.state().q).cwiseAbs().maxCoeff(), kFiveSigma);
  EXPECT_GT((motor - link).cwiseAbs().maxCoeff(), kFiveSigma);  // The gearbox flex shows up here.
}

TEST(SafetyChannel, TheTrackerSeesTheTrueToolNotTheEncoders) {
  stapes::test::verifies("SRS-ARM-008");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(3);
  const SimulatedArm arm(config);
  const stapes::math::Pose truth =
      stapes::kinematics::forward_kinematics(config.model.arm(), arm.state().q);
  const stapes::math::Pose tracked = arm.read_safety_sensors().tracked_tool_pose;
  EXPECT_LT((tracked.translation() - truth.translation()).norm(),
            6.0 * config.tracker.position_noise_m);
  EXPECT_FALSE(tracked.translation().isApprox(truth.translation(), 1e-9));
}

TEST(SafetyChannel, BothChannelsAreStampedWithTheTick) {
  stapes::test::verifies("SRS-ARM-008");
  SimulatedArm arm(SimulatedArmConfig::reference(4));
  EXPECT_EQ(arm.read_sensors().stamp_ticks, 0);
  EXPECT_EQ(arm.read_safety_sensors().stamp_ticks, 0);
  for (int i = 0; i < 7; ++i) {
    arm.tick();
  }
  EXPECT_EQ(arm.read_sensors().stamp_ticks, 7);
  EXPECT_EQ(arm.read_safety_sensors().stamp_ticks, 7);
}

TEST(SafetyChannel, BothChannelsReportTheSameForceSensor) {
  // The robot has one force sensor. The interface documents that this is shared.
  stapes::test::verifies("SRS-ARM-008");
  SimulatedArm arm(SimulatedArmConfig::reference(5));
  stapes::math::Wrench push = stapes::math::Wrench::Zero();
  push(1) = 3.0;
  arm.set_external_wrench(push);
  arm.tick();
  EXPECT_EQ(arm.read_sensors().tool_wrench, arm.read_safety_sensors().tool_wrench);
  EXPECT_GT(arm.read_safety_sensors().tool_wrench.norm(), 1.0);
}

TEST(SafetyChannel, TheNewSensorsDoNotDisturbTheOldOnes) {
  // Each noise source has its own stream, so the motor encoders read the same with or without
  // noise on the safety channel.
  stapes::test::verifies("SRS-ARM-008");
  SimulatedArmConfig quiet = SimulatedArmConfig::reference(6);
  quiet.link_encoder.noise_stddev_rad = 0.0;
  quiet.tracker = stapes::plant::TrackerParams{0.0, 0.0};
  SimulatedArm first(SimulatedArmConfig::reference(6));
  SimulatedArm second(quiet);
  for (int i = 0; i < 10; ++i) {
    first.tick();
    second.tick();
    ASSERT_EQ(first.read_sensors().joint_positions_rad, second.read_sensors().joint_positions_rad);
  }
}

// ---- Brakes ----------------------------------------------------------------------------------

TEST(Brakes, RemoveMotorTorqueAndIgnoreLaterCommands) {
  stapes::test::verifies("SRS-ARM-009");
  SimulatedArm arm(SimulatedArmConfig::reference(7));
  arm.command_torques(JointVector::Constant(1.0));
  ASSERT_FALSE(arm.applied_torques().isZero());
  ASSERT_FALSE(arm.brakes_engaged());

  arm.engage_brakes();
  EXPECT_TRUE(arm.brakes_engaged());
  EXPECT_TRUE(arm.applied_torques().isZero());
  arm.command_torques(JointVector::Constant(1.0));
  EXPECT_TRUE(arm.applied_torques().isZero());
}

TEST(Brakes, StopAMovingMotorWithinFiveTicks) {
  // A brake has a finite torque, so a motor takes J * v / torque to stop. That is under 2 ms for
  // the fastest joint here, with the rotor inertias and speeds of the reference arm.
  stapes::test::verifies("SRS-ARM-009");
  SimulatedArm arm(SimulatedArmConfig::reference(8));
  for (int i = 0; i < 20; ++i) {
    arm.command_torques(JointVector::Constant(0.5));
    arm.tick();
  }
  ASSERT_GT(arm.state().motor_velocity.cwiseAbs().maxCoeff(), 0.01);
  arm.engage_brakes();
  for (int i = 0; i < 5; ++i) {
    arm.tick();
  }
  EXPECT_LT(arm.state().motor_velocity.cwiseAbs().maxCoeff(), 1e-9);
}

TEST(Brakes, HoldTheArmAgainstGravity) {
  stapes::test::verifies("SRS-ARM-009");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(9);
  SimulatedArm arm(config);
  arm.engage_brakes();
  const JointVector motors = arm.state().motor_angle;
  for (int i = 0; i < 2000; ++i) {
    arm.tick();
  }
  EXPECT_LT((arm.state().motor_angle - motors).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((arm.state().q - config.initial_q).cwiseAbs().maxCoeff(), 1e-3);
  EXPECT_LT(arm.state().qd.cwiseAbs().maxCoeff(), 1e-2);
}

TEST(Brakes, AreWhatHoldsTheArm) {
  // Negative control: with the same stop but no brake torque, the arm falls.
  stapes::test::verifies("SRS-ARM-009");
  SimulatedArmConfig config = SimulatedArmConfig::reference(10);
  config.brake_torque_nm.setZero();
  SimulatedArm arm(config);
  arm.engage_brakes();
  for (int i = 0; i < 500; ++i) {
    arm.tick();
  }
  EXPECT_GT((arm.state().q - config.initial_q).cwiseAbs().maxCoeff(), 0.05);
}

TEST(Brakes, AreLimitedToTheirRatedTorque) {
  // A brake weaker than the load slips, instead of holding by magic.
  stapes::test::verifies("SRS-ARM-009");
  const SimulatedArmConfig reference = SimulatedArmConfig::reference(11);
  const JointVector load = stapes::dynamics::gravity_torques(reference.model, reference.initial_q);
  Eigen::Index loaded = 0;
  load.cwiseAbs().maxCoeff(&loaded);

  SimulatedArmConfig weak = reference;
  weak.brake_torque_nm(loaded) = 0.2 * std::abs(load(loaded));
  SimulatedArm arm(weak);
  arm.engage_brakes();
  const double start = arm.state().motor_angle(loaded);
  for (int i = 0; i < 200; ++i) {
    arm.tick();
  }
  EXPECT_GT(std::abs(arm.state().motor_angle(loaded) - start), 1e-3);
}

TEST(Brakes, CanBeReleased) {
  stapes::test::verifies("SRS-ARM-009");
  SimulatedArm arm(SimulatedArmConfig::reference(12));
  arm.engage_brakes();
  arm.tick();
  arm.release_brakes();
  EXPECT_FALSE(arm.brakes_engaged());
  arm.command_torques(JointVector::Constant(1.0));
  EXPECT_FALSE(arm.applied_torques().isZero());
}

TEST(Brakes, ARunWithBrakesHashesDifferently) {
  stapes::test::verifies("SRS-ARM-009");
  const auto hash_of = [](bool brake) {
    SimulatedArm arm(SimulatedArmConfig::reference(13));
    if (brake) {
      arm.engage_brakes();
    }
    stapes::base::StateHash hash;
    arm.add_to_hash(hash);
    return hash.value();
  };
  EXPECT_NE(hash_of(true), hash_of(false));
}

TEST(Brakes, RejectAnInvalidRating) {
  stapes::test::verifies("SRS-ARM-009");
  SimulatedArmConfig negative = SimulatedArmConfig::reference(14);
  negative.brake_torque_nm(2) = -1.0;
  EXPECT_THROW(SimulatedArm{negative}, std::invalid_argument);
  SimulatedArmConfig not_finite = SimulatedArmConfig::reference(14);
  not_finite.brake_torque_nm(2) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SimulatedArm{not_finite}, std::invalid_argument);
}

}  // namespace
