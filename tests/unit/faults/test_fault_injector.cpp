#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/faults/fault_injector.hpp"
#include "stapes/hardware/robot_interface.hpp"
#include "stapes/hardware/safety_interface.hpp"

namespace {

using stapes::faults::Fault;
using stapes::faults::FaultInjector;
using stapes::faults::FaultKind;
using stapes::hardware::SafetySample;
using stapes::hardware::SensorSample;
using stapes::kinematics::JointVector;

// A robot whose every reading is its tick count, so a late or frozen value is easy to recognise.
class CountingRobot final : public stapes::hardware::RobotInterface,
                            public stapes::hardware::SafetyInterface {
 public:
  void tick() { ++tick_; }

  [[nodiscard]] SensorSample read_sensors() const override {
    return SensorSample{JointVector::Constant(static_cast<double>(tick_)),
                        stapes::math::Wrench::Constant(static_cast<double>(tick_)), tick_};
  }
  void command_torques(const JointVector& torques_nm) override { commanded_ = torques_nm; }

  [[nodiscard]] SafetySample read_safety_sensors() const override {
    stapes::math::Pose pose = stapes::math::Pose::Identity();
    pose.translation().setConstant(static_cast<double>(tick_));
    return SafetySample{JointVector::Constant(static_cast<double>(tick_)), pose,
                        stapes::math::Wrench::Zero(), tick_};
  }
  void engage_brakes() override { brakes_ = true; }
  void release_brakes() override { brakes_ = false; }

  [[nodiscard]] const JointVector& commanded() const { return commanded_; }
  [[nodiscard]] bool brakes() const { return brakes_; }

 private:
  std::int64_t tick_{0};
  JointVector commanded_{JointVector::Zero()};
  bool brakes_{false};
};

void advance(CountingRobot& robot, FaultInjector& injector, int ticks) {
  for (int i = 0; i < ticks; ++i) {
    robot.tick();
    injector.update();
  }
}

TEST(FaultInjector, WithNoFaultItChangesNothing) {
  stapes::test::verifies("SRS-FLT-001");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{});
  EXPECT_FALSE(injector.active());
  advance(robot, injector, 10);
  const JointVector torque = JointVector::Constant(3.0);
  injector.command_torques(torque);
  EXPECT_EQ(robot.commanded(), torque);
  EXPECT_EQ(injector.read_sensors().joint_positions_rad, robot.read_sensors().joint_positions_rad);
  EXPECT_EQ(injector.read_sensors().stamp_ticks, 10);
  EXPECT_EQ(injector.read_safety_sensors().link_positions_rad,
            robot.read_safety_sensors().link_positions_rad);
  EXPECT_FALSE(injector.active());
}

TEST(FaultInjector, ReadsTheRobotOnConstruction) {
  stapes::test::verifies("SRS-FLT-001");
  CountingRobot robot;
  robot.tick();
  robot.tick();
  robot.tick();
  const FaultInjector injector(robot, robot, Fault{});
  EXPECT_EQ(injector.read_sensors().stamp_ticks, 3);
  EXPECT_EQ(injector.read_safety_sensors().stamp_ticks, 3);
}

TEST(FaultInjector, DoesNothingBeforeItsStartTick) {
  stapes::test::verifies("SRS-FLT-001");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::frozen_motor_encoder, 5, 2, 0});
  advance(robot, injector, 4);
  EXPECT_FALSE(injector.active());
  EXPECT_DOUBLE_EQ(injector.read_sensors().joint_positions_rad(2), 4.0);
  advance(robot, injector, 1);
  EXPECT_TRUE(injector.active());
}

TEST(FaultInjector, AFrozenMotorEncoderRepeatsItsValueWithAFreshStamp) {
  stapes::test::verifies("SRS-FLT-002");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::frozen_motor_encoder, 5, 2, 0});
  advance(robot, injector, 20);
  const SensorSample reading = injector.read_sensors();
  EXPECT_DOUBLE_EQ(reading.joint_positions_rad(2), 4.0);  // The last value before the fault.
  EXPECT_DOUBLE_EQ(reading.joint_positions_rad(1), 20.0);
  EXPECT_DOUBLE_EQ(reading.joint_positions_rad(3), 20.0);
  EXPECT_EQ(reading.stamp_ticks, 20);
  EXPECT_DOUBLE_EQ(injector.read_safety_sensors().link_positions_rad(2), 20.0);
}

TEST(FaultInjector, AFrozenLinkEncoderAffectsOnlyTheSafetyChannel) {
  stapes::test::verifies("SRS-FLT-003");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::frozen_link_encoder, 5, 4, 0});
  advance(robot, injector, 20);
  const SafetySample reading = injector.read_safety_sensors();
  EXPECT_DOUBLE_EQ(reading.link_positions_rad(4), 4.0);
  EXPECT_DOUBLE_EQ(reading.link_positions_rad(0), 20.0);
  EXPECT_EQ(reading.stamp_ticks, 20);
  EXPECT_DOUBLE_EQ(injector.read_sensors().joint_positions_rad(4), 20.0);
}

TEST(FaultInjector, AFrozenTrackerRepeatsItsPoseWithAFreshStamp) {
  stapes::test::verifies("SRS-FLT-004");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::frozen_tracker, 5, 0, 0});
  advance(robot, injector, 20);
  const SafetySample reading = injector.read_safety_sensors();
  EXPECT_DOUBLE_EQ(reading.tracked_tool_pose.translation().x(), 4.0);
  EXPECT_DOUBLE_EQ(reading.link_positions_rad(0), 20.0);
  EXPECT_EQ(reading.stamp_ticks, 20);
}

TEST(FaultInjector, AStuckActuatorKeepsItsLastTorqueAndIgnoresNewOnes) {
  stapes::test::verifies("SRS-FLT-005");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::stuck_actuator, 5, 1, 0});
  advance(robot, injector, 4);
  injector.command_torques(JointVector::Constant(7.0));
  advance(robot, injector, 1);
  injector.command_torques(JointVector::Constant(-3.0));
  EXPECT_DOUBLE_EQ(robot.commanded()(1), 7.0);
  EXPECT_DOUBLE_EQ(robot.commanded()(0), -3.0);
  EXPECT_DOUBLE_EQ(robot.commanded()(2), -3.0);
  injector.command_torques(JointVector::Constant(9.0));
  EXPECT_DOUBLE_EQ(robot.commanded()(1), 7.0);
}

TEST(FaultInjector, ACommsDelayMakesReadingsLateAndShowsItInTheStamp) {
  stapes::test::verifies("SRS-FLT-006");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::comms_delay, 5, 0, 3});
  advance(robot, injector, 20);
  const SensorSample reading = injector.read_sensors();
  EXPECT_EQ(reading.stamp_ticks, 17);
  EXPECT_DOUBLE_EQ(reading.joint_positions_rad(0), 17.0);
  EXPECT_DOUBLE_EQ(reading.tool_wrench(0), 17.0);
}

TEST(FaultInjector, ACommsDelayMakesCommandsLate) {
  stapes::test::verifies("SRS-FLT-006");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::comms_delay, 0, 0, 3});
  for (int tick = 0; tick < 20; ++tick) {
    injector.command_torques(JointVector::Constant(static_cast<double>(tick)));
    if (tick >= 3) {
      EXPECT_DOUBLE_EQ(robot.commanded()(0), static_cast<double>(tick - 3));
    } else {
      EXPECT_DOUBLE_EQ(robot.commanded()(0), 0.0);
    }
    advance(robot, injector, 1);
  }
}

TEST(FaultInjector, ACommsDelayDoesNotTouchTheSafetyChannel) {
  stapes::test::verifies("SRS-FLT-006");
  CountingRobot robot;
  FaultInjector injector(robot, robot, Fault{FaultKind::comms_delay, 0, 0, 10});
  advance(robot, injector, 20);
  EXPECT_EQ(injector.read_safety_sensors().stamp_ticks, 20);
}

TEST(FaultInjector, DelaysStayCorrectAcrossTheBufferWrapAround) {
  stapes::test::verifies("SRS-FLT-006");
  CountingRobot robot;
  FaultInjector injector(robot, robot,
                         Fault{FaultKind::comms_delay, 0, 0, FaultInjector::kMaxDelayTicks});
  for (int tick = 0; tick < 1000; ++tick) {
    injector.command_torques(JointVector::Constant(static_cast<double>(tick)));
    if (tick >= FaultInjector::kMaxDelayTicks) {
      ASSERT_DOUBLE_EQ(robot.commanded()(0),
                       static_cast<double>(tick - FaultInjector::kMaxDelayTicks));
    }
    advance(robot, injector, 1);
    if (tick + 1 >= FaultInjector::kMaxDelayTicks) {
      ASSERT_EQ(injector.read_sensors().stamp_ticks, tick + 1 - FaultInjector::kMaxDelayTicks);
    }
  }
}

TEST(FaultInjector, BrakesAlwaysPassStraightThrough) {
  stapes::test::verifies("SRS-FLT-007");
  for (const FaultKind kind :
       {FaultKind::none, FaultKind::frozen_motor_encoder, FaultKind::frozen_link_encoder,
        FaultKind::frozen_tracker, FaultKind::stuck_actuator, FaultKind::comms_delay}) {
    CountingRobot robot;
    FaultInjector injector(robot, robot, Fault{kind, 0, 0, 50});
    advance(robot, injector, 5);
    injector.engage_brakes();
    EXPECT_TRUE(robot.brakes());
    injector.release_brakes();
    EXPECT_FALSE(robot.brakes());
  }
}

TEST(FaultInjector, RejectsAJointOrDelayOutOfRange) {
  stapes::test::verifies("SRS-FLT-008");
  CountingRobot robot;
  EXPECT_THROW(FaultInjector(robot, robot, Fault{FaultKind::stuck_actuator, 0, 6, 0}),
               std::invalid_argument);
  EXPECT_THROW(FaultInjector(robot, robot, Fault{FaultKind::comms_delay, 0, 0, -1}),
               std::invalid_argument);
  EXPECT_THROW(
      FaultInjector(robot, robot,
                    Fault{FaultKind::comms_delay, 0, 0, FaultInjector::kMaxDelayTicks + 1}),
      std::invalid_argument);
  EXPECT_NO_THROW(FaultInjector(
      robot, robot, Fault{FaultKind::comms_delay, 0, 0, FaultInjector::kMaxDelayTicks}));
  EXPECT_NO_THROW(FaultInjector(robot, robot, Fault{FaultKind::stuck_actuator, 0, 5, 0}));
}

}  // namespace
