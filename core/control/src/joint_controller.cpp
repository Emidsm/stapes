#include "stapes/control/joint_controller.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "stapes/dynamics/rigid_body_dynamics.hpp"

namespace stapes::control {

JointGains reference_gains() {
  constexpr double kFilter = 0.002;
  return JointGains{
      PidGains{2000.0, 10000.0, 100.0, 60.0, kFilter},
      PidGains{2000.0, 10000.0, 100.0, 60.0, kFilter},
      PidGains{900.0, 4500.0, 40.0, 30.0, kFilter},
      PidGains{120.0, 600.0, 5.0, 8.0, kFilter},
      PidGains{120.0, 600.0, 5.0, 8.0, kFilter},
      PidGains{50.0, 250.0, 2.0, 4.0, kFilter},
  };
}

GearboxEstimates reference_gearbox_estimates() {
  constexpr double kBacklash = 1e-4;
  return GearboxEstimates{
      GearboxEstimate{30000.0, kBacklash}, GearboxEstimate{30000.0, kBacklash},
      GearboxEstimate{15000.0, kBacklash}, GearboxEstimate{2000.0, kBacklash},
      GearboxEstimate{2000.0, kBacklash},  GearboxEstimate{1000.0, kBacklash},
  };
}

JointController::JointController(const dynamics::DynamicModel& model, const JointGains& gains,
                                 const GearboxEstimates& gearboxes, double period_s)
    : model_(model),
      gearboxes_(gearboxes),
      pids_{PidController(gains[0], period_s), PidController(gains[1], period_s),
            PidController(gains[2], period_s), PidController(gains[3], period_s),
            PidController(gains[4], period_s), PidController(gains[5], period_s)} {
  for (const GearboxEstimate& gearbox : gearboxes_) {
    if (!(gearbox.stiffness_nm_per_rad > 0.0) || !(gearbox.backlash_half_width_rad >= 0.0)) {
      throw std::invalid_argument("gearbox estimate needs positive stiffness");
    }
  }
}

kinematics::JointVector JointController::motor_target(
    const kinematics::JointVector& target) const noexcept {
  // Below this load the teeth may be on either side, so the backlash term fades out rather
  // than flipping sign, which would make the motor target jump.
  constexpr double kBacklashBlendTorque = 0.05;

  const kinematics::JointVector load = dynamics::gravity_torques(model_, target);
  kinematics::JointVector motor = target;
  for (std::size_t i = 0; i < gearboxes_.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    motor(row) +=
        (load(row) / gearboxes_[i].stiffness_nm_per_rad) +
        (gearboxes_[i].backlash_half_width_rad * std::tanh(load(row) / kBacklashBlendTorque));
  }
  return motor;
}

kinematics::JointVector JointController::update(const kinematics::JointVector& target,
                                                const kinematics::JointVector& measured) noexcept {
  const kinematics::JointVector motor = motor_target(target);
  const kinematics::JointVector gravity = dynamics::gravity_torques(model_, measured);
  kinematics::JointVector torques;
  for (std::size_t i = 0; i < pids_.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    torques(row) = pids_[i].update(motor(row), measured(row), gravity(row));
  }
  return torques;
}

void JointController::reset() noexcept {
  for (PidController& pid : pids_) {
    pid.reset();
  }
}

}  // namespace stapes::control
