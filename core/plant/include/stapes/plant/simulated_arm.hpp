#pragma once

#include <array>
#include <cstdint>

#include "stapes/base/sim_clock.hpp"
#include "stapes/base/state_hash.hpp"
#include "stapes/dynamics/dynamic_model.hpp"
#include "stapes/hardware/robot_interface.hpp"
#include "stapes/plant/actuator.hpp"
#include "stapes/plant/encoder.hpp"
#include "stapes/plant/force_sensor.hpp"

namespace stapes::plant {

struct SimulatedArmConfig {
  dynamics::DynamicModel model;
  Actuators actuators;
  EncoderParams encoder;
  ForceSensorParams force_sensor;
  base::SimDuration physics_step;
  base::SimDuration control_period;
  kinematics::JointVector initial_q;
  std::uint64_t seed;

  /// The reference arm with 10 kHz physics, a 1 kHz control period, and 20-bit encoders.
  [[nodiscard]] static SimulatedArmConfig reference(std::uint64_t seed);
};

/// The true state of the simulated robot. Control code never sees this.
struct ArmState {
  kinematics::JointVector q;
  kinematics::JointVector qd;
  kinematics::JointVector motor_angle;
  kinematics::JointVector motor_velocity;
};

/// The simulated robot: rigid-body arm, gearboxes, motors and sensors.
///
/// Each joint has two masses, the motor and the link, coupled by the gearbox. Encoders
/// sit on the motor side, so the controller cannot see backlash or gearbox deflection,
/// as on a real geared arm. Physics uses semi-implicit Euler, a standard choice for stiff
/// springs like these gearboxes because it does not steadily add energy to an oscillator.
class SimulatedArm final : public hardware::RobotInterface {
 public:
  /// @throws std::invalid_argument if the control period is not a positive whole multiple
  ///         of a positive physics step, or the initial configuration is not finite.
  explicit SimulatedArm(const SimulatedArmConfig& config);

  [[nodiscard]] hardware::SensorSample read_sensors() const override { return sample_; }

  /// Held until the next command. Saturated at each motor's limit.
  void command_torques(const kinematics::JointVector& torques_nm) override;

  /// A wrench the environment applies at the tool tip, in world coordinates.
  void set_external_wrench(const math::Wrench& world_wrench) noexcept {
    external_wrench_ = world_wrench;
  }

  /// Advances one control period, then samples the sensors.
  void tick() noexcept;

  [[nodiscard]] const ArmState& state() const noexcept { return state_; }
  [[nodiscard]] const kinematics::JointVector& applied_torques() const noexcept {
    return applied_torques_;
  }
  [[nodiscard]] const base::SimClock& clock() const noexcept { return clock_; }

  /// Adds the true state and the latest readings to `hash`.
  void add_to_hash(base::StateHash& hash) const noexcept;

 private:
  void step_physics(double step_s) noexcept;
  void sample_sensors() noexcept;

  dynamics::DynamicModel model_;
  Actuators actuators_;
  std::array<Encoder, kinematics::kJointCount> encoders_;
  ForceSensor force_sensor_;
  base::SimClock clock_;
  int physics_steps_per_tick_;
  double physics_step_s_;
  double control_period_s_;

  ArmState state_;
  kinematics::JointVector applied_torques_{kinematics::JointVector::Zero()};
  math::Wrench external_wrench_{math::Wrench::Zero()};
  hardware::SensorSample sample_{};
};

}  // namespace stapes::plant
