#pragma once

#include <array>

#include "stapes/control/pid_controller.hpp"
#include "stapes/dynamics/dynamic_model.hpp"
#include "stapes/kinematics/arm_model.hpp"

namespace stapes::control {

using JointGains = std::array<PidGains, kinematics::kJointCount>;

/// What the controller believes about one gearbox. These are its own estimates: it has no
/// access to the simulated robot's true parameters.
struct GearboxEstimate {
  double stiffness_nm_per_rad;
  double backlash_half_width_rad;
};

using GearboxEstimates = std::array<GearboxEstimate, kinematics::kJointCount>;

/// Gains tuned for the reference arm at a 1 kHz control period.
[[nodiscard]] JointGains reference_gains();

/// Gearbox data for the reference arm, as a datasheet or a calibration would supply it.
[[nodiscard]] GearboxEstimates reference_gearbox_estimates();

/// One PID per joint, plus model-based gravity and gearbox compensation.
///
/// Encoders are on the motor side, so a gearbox that flexes under load makes the link sag
/// with no change in the reading. To put the link on target, the controller aims the motor
/// past it by the deflection it predicts from the gravity load.
///
/// The controller uses its own copy of the model, which need not match the plant exactly.
/// The PID integral absorbs torque errors at the motor, but it cannot see an error in the
/// predicted deflection, since that lies beyond the encoder.
class JointController {
 public:
  JointController(const dynamics::DynamicModel& model, const JointGains& gains,
                  const GearboxEstimates& gearboxes, double period_s);

  /// Where the motors should be for the links to reach `target`, given gravity alone.
  [[nodiscard]] kinematics::JointVector motor_target(
      const kinematics::JointVector& target) const noexcept;

  /// Torques that drive the measured joint positions toward `target`.
  [[nodiscard]] kinematics::JointVector update(const kinematics::JointVector& target,
                                               const kinematics::JointVector& measured) noexcept;

  void reset() noexcept;

 private:
  dynamics::DynamicModel model_;
  GearboxEstimates gearboxes_;
  std::array<PidController, kinematics::kJointCount> pids_;
};

}  // namespace stapes::control
