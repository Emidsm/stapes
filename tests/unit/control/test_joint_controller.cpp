#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "closed_loop_workload.hpp"
#include "requirement.hpp"
#include "stapes/control/joint_controller.hpp"
#include "stapes/dynamics/rigid_body_dynamics.hpp"

namespace {

using stapes::control::GearboxEstimates;
using stapes::control::JointController;
using stapes::control::reference_gains;
using stapes::control::reference_gearbox_estimates;
using stapes::dynamics::DynamicModel;
using stapes::kinematics::JointVector;
using stapes::kinematics::reference_home;

constexpr double kPeriod = 0.001;
const DynamicModel kModel = DynamicModel::reference();

// Stands in for real hardware: nothing about it is simulated physics. The control code
// runs against it unchanged, because it depends only on the interface.
class ScriptedRobot final : public stapes::hardware::RobotInterface {
 public:
  explicit ScriptedRobot(const JointVector& reading) { sample_.joint_positions_rad = reading; }

  [[nodiscard]] stapes::hardware::SensorSample read_sensors() const override { return sample_; }
  void command_torques(const JointVector& torques) override { commanded_ = torques; }

  [[nodiscard]] const JointVector& commanded() const { return commanded_; }

 private:
  JointVector commanded_{JointVector::Zero()};
  stapes::hardware::SensorSample sample_{JointVector::Zero(), stapes::math::Wrench::Zero()};
};

TEST(JointController, RunsAgainstAnyImplementationOfTheHardwareInterface) {
  stapes::test::verifies("SRS-CTL-006");
  const JointVector home = reference_home();
  ScriptedRobot robot(home);
  JointController controller(kModel, reference_gains(), reference_gearbox_estimates(), kPeriod);
  JointController same(kModel, reference_gains(), reference_gearbox_estimates(), kPeriod);

  stapes::test::control_step(robot, controller, home);
  EXPECT_EQ(robot.commanded(), same.update(home, home));
  EXPECT_FALSE(robot.commanded().isZero());
}

TEST(JointController, CommandsExactlyTheGravityTorqueWhenOnTarget) {
  stapes::test::verifies("SRS-CTL-006");
  JointController controller(kModel, reference_gains(), reference_gearbox_estimates(), kPeriod);
  const JointVector target = reference_home();
  const JointVector on_target = controller.motor_target(target);
  EXPECT_TRUE(controller.update(target, on_target)
                  .isApprox(stapes::dynamics::gravity_torques(kModel, on_target), 1e-12));
}

TEST(JointController, AimsTheMotorPastTheLinkByThePredictedDeflection) {
  stapes::test::verifies("SRS-CTL-007");
  const GearboxEstimates gearboxes = reference_gearbox_estimates();
  const JointController controller(kModel, reference_gains(), gearboxes, kPeriod);
  const JointVector target = reference_home();
  const JointVector load = stapes::dynamics::gravity_torques(kModel, target);
  const JointVector motor = controller.motor_target(target);

  // Shoulder and elbow carry real loads, far above the blend torque: stiffness plus backlash.
  for (const Eigen::Index joint : {1, 2}) {
    const auto index = static_cast<std::size_t>(joint);
    ASSERT_GT(std::abs(load(joint)), 1.0);
    const double expected = (load(joint) / gearboxes[index].stiffness_nm_per_rad) +
                            std::copysign(gearboxes[index].backlash_half_width_rad, load(joint));
    EXPECT_NEAR(motor(joint) - target(joint), expected, 1e-12) << "joint " << joint;
  }
  // The vertical base axis carries no load, so its teeth have no preferred side.
  EXPECT_NEAR(motor(0), target(0), 1e-12);
}

TEST(JointController, HoldsOutputsWithinEachJointsLimit) {
  stapes::test::verifies("SRS-CTL-003");
  const stapes::control::JointGains gains = reference_gains();
  JointController controller(kModel, gains, reference_gearbox_estimates(), kPeriod);
  const JointVector torques =
      controller.update(reference_home() + JointVector::Constant(3.0), reference_home());
  for (std::size_t i = 0; i < gains.size(); ++i) {
    EXPECT_LE(std::abs(torques(static_cast<Eigen::Index>(i))), gains[i].output_limit);
  }
}

TEST(JointController, ANaNMeasurementCommandsNoTorqueOnAnyJoint) {
  // Gravity compensation depends on every joint angle, so one bad encoder invalidates all.
  stapes::test::verifies("SRS-CTL-004");
  JointController controller(kModel, reference_gains(), reference_gearbox_estimates(), kPeriod);
  JointVector measured = reference_home();
  measured(3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(controller.update(reference_home(), measured).isZero());
}

TEST(JointController, ResetClearsEveryJoint) {
  stapes::test::verifies("SRS-CTL-005");
  JointController fresh(kModel, reference_gains(), reference_gearbox_estimates(), kPeriod);
  JointController used(kModel, reference_gains(), reference_gearbox_estimates(), kPeriod);
  const JointVector far = reference_home() + JointVector::Constant(0.001);
  for (int i = 0; i < 100; ++i) {
    static_cast<void>(used.update(far, reference_home()));
  }
  used.reset();
  EXPECT_EQ(used.update(far, reference_home()), fresh.update(far, reference_home()));
}

TEST(JointController, RejectsInvalidGearboxEstimates) {
  stapes::test::verifies("SRS-CTL-005");
  GearboxEstimates bad = reference_gearbox_estimates();
  bad[2].stiffness_nm_per_rad = 0.0;
  EXPECT_THROW(JointController(kModel, reference_gains(), bad, kPeriod), std::invalid_argument);
  bad = reference_gearbox_estimates();
  bad[2].backlash_half_width_rad = -1.0;
  EXPECT_THROW(JointController(kModel, reference_gains(), bad, kPeriod), std::invalid_argument);
}

}  // namespace
