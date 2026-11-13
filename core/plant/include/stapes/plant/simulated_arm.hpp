#pragma once

#include <array>
#include <cstdint>

#include "stapes/base/sim_clock.hpp"
#include "stapes/base/state_hash.hpp"
#include "stapes/dynamics/dynamic_model.hpp"
#include "stapes/hardware/robot_interface.hpp"
#include "stapes/hardware/safety_interface.hpp"
#include "stapes/plant/actuator.hpp"
#include "stapes/plant/encoder.hpp"
#include "stapes/plant/force_sensor.hpp"
#include "stapes/plant/optical_tracker.hpp"

namespace stapes::plant {

struct SimulatedArmConfig {
  dynamics::DynamicModel model;
  Actuators actuators;
  EncoderParams encoder;
  EncoderParams link_encoder;
  ForceSensorParams force_sensor;
  TrackerParams tracker;
  kinematics::JointVector brake_torque_nm;
  base::SimDuration physics_step;
  base::SimDuration control_period;
  kinematics::JointVector initial_q;  ///< The links start here, at rest, carrying gravity.
  std::uint64_t seed;

  /// The reference arm with 10 kHz physics, a 1 kHz control period, 20-bit motor encoders and
  /// 19-bit link encoders.
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
///
/// The safety channel has its own sensors: link encoders and an optical tracker.
class SimulatedArm final : public hardware::RobotInterface, public hardware::SafetyInterface {
 public:
  /// @throws std::invalid_argument if the control period is not a positive whole multiple
  ///         of a positive physics step, the initial configuration is not finite, or a brake
  ///         torque is negative or not finite.
  explicit SimulatedArm(const SimulatedArmConfig& config);

  [[nodiscard]] hardware::SensorSample read_sensors() const override { return sample_; }

  /// Held until the next command. Saturated at each motor's limit. Has no effect while the
  /// brakes are engaged.
  void command_torques(const kinematics::JointVector& torques_nm) override;

  [[nodiscard]] hardware::SafetySample read_safety_sensors() const override {
    return safety_sample_;
  }
  /// Cuts motor torque and applies a holding brake on each motor, limited to its brake torque.
  /// The links are still free to move as far as the gearbox lets them.
  void engage_brakes() override;
  void release_brakes() override { brakes_engaged_ = false; }
  [[nodiscard]] bool brakes_engaged() const noexcept { return brakes_engaged_; }

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
  std::array<Encoder, kinematics::kJointCount> link_encoders_;
  ForceSensor force_sensor_;
  OpticalTracker tracker_;
  kinematics::JointVector brake_torque_nm_;
  base::SimClock clock_;
  int physics_steps_per_tick_;
  double physics_step_s_;
  double control_period_s_;

  ArmState state_;
  kinematics::JointVector applied_torques_{kinematics::JointVector::Zero()};
  math::Wrench external_wrench_{math::Wrench::Zero()};
  bool brakes_engaged_{false};
  hardware::SensorSample sample_{};
  hardware::SafetySample safety_sample_{};
};

}  // namespace stapes::plant
