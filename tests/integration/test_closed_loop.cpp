#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "closed_loop_workload.hpp"
#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"

namespace {

using stapes::control::GearboxEstimates;
using stapes::control::JointController;
using stapes::control::reference_gains;
using stapes::control::reference_gearbox_estimates;
using stapes::kinematics::JointVector;
using stapes::plant::SimulatedArm;
using stapes::plant::SimulatedArmConfig;

constexpr double kPeriod = 0.001;
constexpr double kMicrometresPerMetre = 1e6;

// True distance between the tool tip and where it should be. The controller never sees this.
double tip_error_um(const SimulatedArmConfig& config, const SimulatedArm& arm,
                    const JointVector& target) {
  const stapes::kinematics::ArmModel& model = config.model.arm();
  const Eigen::Vector3d actual =
      stapes::kinematics::forward_kinematics(model, arm.state().q).translation();
  const Eigen::Vector3d wanted =
      stapes::kinematics::forward_kinematics(model, target).translation();
  return (actual - wanted).norm() * kMicrometresPerMetre;
}

TEST(ClosedLoop, HoldsAPoseAgainstGravity) {
  stapes::test::verifies("SRS-CTL-008");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(1);
  SimulatedArm arm(config);
  JointController controller(config.model, reference_gains(), reference_gearbox_estimates(),
                             kPeriod);
  const JointVector target = config.initial_q;

  for (int i = 0; i < 2000; ++i) {
    stapes::test::control_step(arm, controller, target);
    arm.tick();
  }
  // Worst case over a further second, rather than one lucky instant.
  double worst_um = 0.0;
  for (int i = 0; i < 1000; ++i) {
    stapes::test::control_step(arm, controller, target);
    arm.tick();
    worst_um = std::max(worst_um, tip_error_um(config, arm, target));
  }
  RecordProperty("hold_tip_error_um", std::to_string(worst_um));
  EXPECT_LT(worst_um, 50.0);
}

TEST(ClosedLoop, SettlesAfterAStepWithBoundedOvershoot) {
  stapes::test::verifies("SRS-CTL-009");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(2);
  SimulatedArm arm(config);
  JointController controller(config.model, reference_gains(), reference_gearbox_estimates(),
                             kPeriod);
  JointVector target = config.initial_q;
  constexpr double kStep = 0.05;
  target.array() += kStep;

  constexpr double kTipToleranceUm = 200.0;
  constexpr int kTicks = 3000;
  int last_outside = 0;
  double overshoot = 0.0;
  double worst_late_um = 0.0;
  for (int i = 1; i <= kTicks; ++i) {
    stapes::test::control_step(arm, controller, target);
    arm.tick();
    const double error_um = tip_error_um(config, arm, target);
    if (error_um > kTipToleranceUm) {
      last_outside = i;
    }
    overshoot = std::max(overshoot, (arm.state().q - target).maxCoeff());
    if (i > kTicks - 500) {
      worst_late_um = std::max(worst_late_um, error_um);
    }
  }
  RecordProperty("step_settle_ms", last_outside);
  RecordProperty("step_overshoot_mrad", std::to_string(overshoot * 1e3));
  RecordProperty("step_final_tip_um", std::to_string(worst_late_um));
  EXPECT_LT(last_outside, 2000) << "still outside 0.2 mm after 2 s";
  EXPECT_LT(overshoot, 0.2 * kStep);
  EXPECT_LT(worst_late_um, kTipToleranceUm);
}

TEST(ClosedLoop, GearboxCompensationIsWhatPutsTheTipOnTarget) {
  // Motor-side control alone reaches its target, and the link still sags out of tolerance.
  stapes::test::verifies("SRS-CTL-007");
  GearboxEstimates ignored = reference_gearbox_estimates();
  for (auto& gearbox : ignored) {
    gearbox.stiffness_nm_per_rad = std::numeric_limits<double>::max();
    gearbox.backlash_half_width_rad = 0.0;
  }
  const auto settled_error = [](const GearboxEstimates& gearboxes) {
    const SimulatedArmConfig config = SimulatedArmConfig::reference(3);
    SimulatedArm arm(config);
    JointController controller(config.model, reference_gains(), gearboxes, kPeriod);
    for (int i = 0; i < 3000; ++i) {
      stapes::test::control_step(arm, controller, config.initial_q);
      arm.tick();
    }
    return tip_error_um(config, arm, config.initial_q);
  };
  const double with_compensation = settled_error(reference_gearbox_estimates());
  const double without_compensation = settled_error(ignored);
  RecordProperty("tip_error_with_compensation_um", std::to_string(with_compensation));
  RecordProperty("tip_error_without_compensation_um", std::to_string(without_compensation));
  EXPECT_GT(without_compensation, 200.0);
  EXPECT_LT(with_compensation, 50.0);
}

TEST(ClosedLoop, ToleratesAnImperfectModel) {
  // The controller believes the links weigh 5 % less and the gearboxes are 10 % stiffer.
  stapes::test::verifies("SRS-CTL-010");
  const SimulatedArmConfig config = SimulatedArmConfig::reference(4);
  SimulatedArm arm(config);

  stapes::dynamics::DynamicModel::Links links = config.model.links();
  for (auto& link : links) {
    link.mass_kg *= 0.95;
    link.inertia_kgm2 *= 0.95;
  }
  const stapes::dynamics::DynamicModel believed(config.model.arm(), links, config.model.gravity());
  GearboxEstimates gearboxes = reference_gearbox_estimates();
  for (auto& gearbox : gearboxes) {
    gearbox.stiffness_nm_per_rad *= 1.10;
  }

  JointController controller(believed, reference_gains(), gearboxes, kPeriod);
  const JointVector target = config.initial_q;
  for (int i = 0; i < 3000; ++i) {
    stapes::test::control_step(arm, controller, target);
    arm.tick();
  }
  const double error_um = tip_error_um(config, arm, target);
  RecordProperty("tip_error_imperfect_model_um", std::to_string(error_um));
  EXPECT_LT(error_um, 200.0);
}

TEST(ClosedLoop, SurvivesASteadyExternalForceAndReportsIt) {
  stapes::test::verifies("SRS-CTL-010");
  SimulatedArmConfig config = SimulatedArmConfig::reference(5);
  SimulatedArm arm(config);
  JointController controller(config.model, reference_gains(), reference_gearbox_estimates(),
                             kPeriod);
  const JointVector target = config.initial_q;
  stapes::math::Wrench push = stapes::math::Wrench::Zero();
  push(2) = 5.0;
  arm.set_external_wrench(push);
  for (int i = 0; i < 3000; ++i) {
    stapes::test::control_step(arm, controller, target);
    arm.tick();
  }
  EXPECT_TRUE(arm.state().q.allFinite());
  EXPECT_LT(arm.state().qd.cwiseAbs().maxCoeff(), 0.01);
  EXPECT_NEAR(arm.read_sensors().tool_wrench.head<3>().norm(), 5.0, 0.5);
  // A force the controller does not know about bends the gearboxes, so the tip moves. This
  // is why a real system also needs external position sensing.
  const double error_um = tip_error_um(config, arm, target);
  RecordProperty("tip_error_under_5n_um", std::to_string(error_um));
  EXPECT_GT(error_um, 1.0);
}

}  // namespace
