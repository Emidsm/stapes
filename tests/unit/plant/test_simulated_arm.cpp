#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/dynamics/rigid_body_dynamics.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/plant/simulated_arm.hpp"

namespace {

using stapes::base::StateHash;
using stapes::kinematics::JointVector;
using stapes::math::Wrench;
using stapes::plant::SimulatedArm;
using stapes::plant::SimulatedArmConfig;

SimulatedArmConfig config_without_gravity() {
  SimulatedArmConfig config = SimulatedArmConfig::reference(1);
  config.model = stapes::dynamics::DynamicModel(config.model.arm(), config.model.links(),
                                                Eigen::Vector3d::Zero());
  return config;
}

std::uint64_t hash_of(const SimulatedArm& arm) {
  StateHash hash;
  arm.add_to_hash(hash);
  return hash.value();
}

TEST(SimulatedArm, FallsUnderGravityWithNoCommand) {
  stapes::test::verifies("SRS-ARM-001");
  SimulatedArm arm(SimulatedArmConfig::reference(1));
  const JointVector start = arm.state().q;
  for (int i = 0; i < 200; ++i) {
    arm.tick();
  }
  EXPECT_GT(std::abs(arm.state().q(1) - start(1)), 1e-3);
  // The vertical base axis carries no gravity load in this configuration.
  EXPECT_NEAR(arm.state().q(0), start(0), 1e-9);
}

TEST(SimulatedArm, StaysAtRestWithoutGravityOrCommand) {
  stapes::test::verifies("SRS-ARM-001");
  SimulatedArm arm(config_without_gravity());
  const JointVector start = arm.state().q;
  for (int i = 0; i < 1000; ++i) {
    arm.tick();
  }
  EXPECT_EQ(arm.state().q, start);
}

TEST(SimulatedArm, MotorMovesThroughTheBacklashBeforeTheLinkMoves) {
  stapes::test::verifies("SRS-ARM-002");
  const SimulatedArmConfig config = config_without_gravity();
  const double backlash = config.actuators[5].gear.backlash_half_width_rad;
  SimulatedArm arm(config);
  const double link_start = arm.state().q(5);
  const double motor_start = arm.state().motor_angle(5);

  JointVector command = JointVector::Zero();
  command(5) = 0.5;  // Above friction, on the lightest joint.
  arm.command_torques(command);

  while (arm.state().motor_angle(5) - motor_start < 0.9 * backlash) {
    arm.tick();
    ASSERT_LT(arm.clock().ticks(), 1000) << "motor never crossed the dead zone";
  }
  EXPECT_EQ(arm.state().q(5), link_start) << "the link moved before the teeth met";

  for (int i = 0; i < 20; ++i) {
    arm.tick();
  }
  EXPECT_GT(arm.state().q(5), link_start) << "the link never moved after the teeth met";
}

TEST(SimulatedArm, EncodersReadTheMotorNotTheLink) {
  // Gravity flexes the gearbox, so motor and link differ. The reading must follow the motor.
  stapes::test::verifies("SRS-ARM-002");
  SimulatedArmConfig config = SimulatedArmConfig::reference(1);
  config.encoder.noise_stddev_rad = 0.0;
  SimulatedArm arm(config);
  for (int i = 0; i < 100; ++i) {
    arm.tick();
  }
  const double resolution = config.encoder.resolution_rad;
  const JointVector reading = arm.read_sensors().joint_positions_rad;
  EXPECT_LE((reading - arm.state().motor_angle).cwiseAbs().maxCoeff(), resolution);
  EXPECT_GT((reading - arm.state().q).cwiseAbs().maxCoeff(), 10.0 * resolution);
}

TEST(SimulatedArm, EachEncoderHasItsOwnNoise) {
  stapes::test::verifies("SRS-ARM-002");
  SimulatedArmConfig config = config_without_gravity();
  config.encoder.noise_stddev_rad = 0.01;
  config.initial_q = JointVector::Constant(0.5);
  const SimulatedArm arm(config);
  const JointVector reading = arm.read_sensors().joint_positions_rad;
  for (Eigen::Index joint = 1; joint < reading.size(); ++joint) {
    EXPECT_NE(reading(joint), reading(0));
  }
}

TEST(SimulatedArm, FrictionHoldsAMotorBelowItsBreakawayTorque) {
  stapes::test::verifies("SRS-ARM-003");
  const SimulatedArmConfig config = config_without_gravity();
  const stapes::plant::FrictionParams& friction = config.actuators[0].friction;
  SimulatedArm arm(config);
  JointVector command = JointVector::Zero();
  command(0) = 0.5 * friction.coulomb_nm;
  arm.command_torques(command);
  for (int i = 0; i < 500; ++i) {
    arm.tick();
  }
  // Friction balances the command at a slow creep, far below the smoothing velocity.
  EXPECT_LT(std::abs(arm.state().motor_velocity(0)), friction.smoothing_velocity_rad_s);
}

TEST(SimulatedArm, SaturatesCommandsAtTheMotorLimit) {
  stapes::test::verifies("SRS-ARM-003");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(1);
  SimulatedArm huge(config);
  SimulatedArm exact(config);
  JointVector limits;
  for (Eigen::Index i = 0; i < limits.size(); ++i) {
    limits(i) = config.actuators[static_cast<std::size_t>(i)].max_torque_nm;
  }
  huge.command_torques(JointVector::Constant(1e9));
  exact.command_torques(limits);
  EXPECT_EQ(huge.applied_torques(), limits);
  for (int i = 0; i < 100; ++i) {
    huge.tick();
    exact.tick();
  }
  EXPECT_EQ(hash_of(huge), hash_of(exact));
}

TEST(SimulatedArm, TreatsANaNCommandAsNoTorqueOnThatJoint) {
  stapes::test::verifies("SRS-ARM-003");
  SimulatedArm arm(SimulatedArmConfig::reference(1));
  JointVector command = JointVector::Constant(1.0);
  command(2) = std::numeric_limits<double>::quiet_NaN();
  arm.command_torques(command);
  EXPECT_EQ(arm.applied_torques()(2), 0.0);
  EXPECT_EQ(arm.applied_torques()(1), 1.0);
  for (int i = 0; i < 100; ++i) {
    arm.tick();
  }
  EXPECT_TRUE(arm.state().q.allFinite());
}

TEST(SimulatedArm, ExternalWrenchActsThroughTheJacobianTranspose) {
  stapes::test::verifies("SRS-ARM-004");
  const SimulatedArmConfig config = config_without_gravity();
  SimulatedArm arm(config);
  Wrench wrench;
  wrench << 0.0, 0.0, 20.0, 0.0, 0.0, 0.0;

  const JointVector q = arm.state().q;
  const JointVector torques =
      stapes::kinematics::geometric_jacobian(config.model.arm(), q).transpose() * wrench;
  const JointVector expected =
      stapes::dynamics::forward_dynamics(config.model, q, JointVector::Zero(), torques);

  arm.set_external_wrench(wrench);
  arm.tick();
  // For a short time the teeth stay apart, so the links accelerate freely.
  EXPECT_TRUE(arm.state().qd.isApprox(expected * 1e-3, 0.02));
}

TEST(SimulatedArm, ReportsTheExternalWrenchInTheToolFrame) {
  stapes::test::verifies("SRS-ARM-004");
  SimulatedArmConfig config = config_without_gravity();
  config.force_sensor = stapes::plant::ForceSensorParams{0.0, 0.0, 0.0, 0.0, 0.0};
  SimulatedArm arm(config);
  Wrench wrench;
  wrench << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
  arm.set_external_wrench(wrench);
  arm.tick();

  const Eigen::Matrix3d tool_to_world =
      stapes::kinematics::forward_kinematics(config.model.arm(), arm.state().q).linear();
  const Wrench reading = arm.read_sensors().tool_wrench;
  EXPECT_TRUE((tool_to_world * reading.head<3>()).isApprox(wrench.head<3>(), 1e-12));
  EXPECT_TRUE((tool_to_world * reading.tail<3>()).isApprox(wrench.tail<3>(), 1e-12));
}

TEST(SimulatedArm, StaysFiniteUnderRepeatedFullTorqueReversals) {
  // The hardest case for the integrator: stiff gearboxes hit repeatedly at full torque.
  stapes::test::verifies("SRS-ARM-005");
  SimulatedArm arm(SimulatedArmConfig::reference(1));
  for (int i = 0; i < 5000; ++i) {
    const double direction = (i / 50) % 2 == 0 ? 1.0 : -1.0;
    arm.command_torques(JointVector::Constant(direction * 1e9));
    arm.tick();
    ASSERT_TRUE(arm.state().q.allFinite()) << "tick " << i;
    ASSERT_LT(arm.state().qd.cwiseAbs().maxCoeff(), 1000.0) << "tick " << i;
  }
}

TEST(SimulatedArm, ClockAdvancesOneControlPeriodPerTick) {
  stapes::test::verifies("SRS-ARM-006");
  SimulatedArm arm(SimulatedArmConfig::reference(1));
  arm.tick();
  arm.tick();
  EXPECT_EQ(arm.clock().ticks(), 2);
  EXPECT_EQ(arm.clock().now(), std::chrono::milliseconds(2));
}

TEST(SimulatedArm, RejectsInvalidConfiguration) {
  stapes::test::verifies("SRS-ARM-006");
  SimulatedArmConfig bad = SimulatedArmConfig::reference(1);
  bad.physics_step = std::chrono::microseconds(300);  // 1 ms is not a whole multiple.
  EXPECT_THROW(SimulatedArm{bad}, std::invalid_argument);

  bad = SimulatedArmConfig::reference(1);
  bad.physics_step = std::chrono::microseconds(0);
  EXPECT_THROW(SimulatedArm{bad}, std::invalid_argument);

  bad = SimulatedArmConfig::reference(1);
  bad.control_period = std::chrono::microseconds(0);
  EXPECT_THROW(SimulatedArm{bad}, std::invalid_argument);

  bad = SimulatedArmConfig::reference(1);
  bad.initial_q(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SimulatedArm{bad}, std::invalid_argument);
}

}  // namespace
