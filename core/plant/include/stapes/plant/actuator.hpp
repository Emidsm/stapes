#pragma once

#include <array>

#include "stapes/kinematics/arm_model.hpp"

namespace stapes::plant {

/// All quantities in this file are on the output side of the gearbox.

/// Friction: Coulomb, plus a Stribeck peak at low speed, plus viscous.
struct FrictionParams {
  double coulomb_nm;
  double stribeck_nm;  ///< Extra friction at breakaway, above Coulomb.
  double stribeck_velocity_rad_s;
  double viscous_nm_s;
  /// Replaces the sign function's jump at zero speed with a steep slope, which the
  /// integrator can follow. The arm creeps very slowly rather than sticking exactly.
  double smoothing_velocity_rad_s;
};

/// Elastic gearbox with a dead zone. Inside the dead zone the teeth do not touch.
struct GearParams {
  double stiffness_nm_per_rad;
  double damping_nm_s;
  double backlash_half_width_rad;
};

struct ActuatorParams {
  double max_torque_nm;
  double rotor_inertia_kgm2;
  FrictionParams friction;
  GearParams gear;
};

using Actuators = std::array<ActuatorParams, kinematics::kJointCount>;

/// Friction torque opposing a motor moving at `velocity`.
[[nodiscard]] double friction_torque(const FrictionParams& params, double velocity) noexcept;

/// Torque the gearbox applies to the link. `deflection` is motor angle minus link angle.
/// Teeth can only push, so damping can never make the contact pull.
[[nodiscard]] double gear_torque(const GearParams& params, double deflection,
                                 double deflection_rate) noexcept;

/// Clamps a command to the motor's limit. A NaN command produces no torque.
[[nodiscard]] double saturate_torque(double command, double max_torque) noexcept;

/// Illustrative values for the reference arm, not those of any real actuator.
[[nodiscard]] Actuators reference_actuators();

}  // namespace stapes::plant
