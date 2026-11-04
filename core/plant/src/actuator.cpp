#include "stapes/plant/actuator.hpp"

#include <algorithm>
#include <cmath>

namespace stapes::plant {

double friction_torque(const FrictionParams& params, double velocity) noexcept {
  const double normalised = velocity / params.stribeck_velocity_rad_s;
  const double dry = params.coulomb_nm + (params.stribeck_nm * std::exp(-normalised * normalised));
  return (dry * std::tanh(velocity / params.smoothing_velocity_rad_s)) +
         (params.viscous_nm_s * velocity);
}

double gear_torque(const GearParams& params, double deflection, double deflection_rate) noexcept {
  if (deflection > params.backlash_half_width_rad) {
    const double penetration = deflection - params.backlash_half_width_rad;
    const double push =
        (params.stiffness_nm_per_rad * penetration) + (params.damping_nm_s * deflection_rate);
    return std::max(push, 0.0);
  }
  if (deflection < -params.backlash_half_width_rad) {
    const double penetration = deflection + params.backlash_half_width_rad;
    const double push =
        (params.stiffness_nm_per_rad * penetration) + (params.damping_nm_s * deflection_rate);
    return std::min(push, 0.0);
  }
  return 0.0;
}

double saturate_torque(double command, double max_torque) noexcept {
  return std::isnan(command) ? 0.0 : std::clamp(command, -max_torque, max_torque);
}

Actuators reference_actuators() {
  constexpr double kBacklash = 1e-4;
  constexpr double kStribeckVelocity = 0.02;
  constexpr double kSmoothingVelocity = 0.005;

  const auto actuator = [](double max_torque, double rotor_inertia, double coulomb, double viscous,
                           double stiffness, double damping) {
    return ActuatorParams{
        max_torque,
        rotor_inertia,
        FrictionParams{coulomb, 0.25 * coulomb, kStribeckVelocity, viscous, kSmoothingVelocity},
        GearParams{stiffness, damping, kBacklash},
    };
  };

  return Actuators{
      actuator(60.0, 0.15, 0.60, 0.20, 30000.0, 5.0),
      actuator(60.0, 0.15, 0.60, 0.20, 30000.0, 5.0),
      actuator(30.0, 0.08, 0.40, 0.10, 15000.0, 2.0),
      actuator(8.0, 0.02, 0.10, 0.02, 2000.0, 0.2),
      actuator(8.0, 0.02, 0.10, 0.02, 2000.0, 0.2),
      actuator(4.0, 0.01, 0.05, 0.01, 1000.0, 0.1),
  };
}

}  // namespace stapes::plant
