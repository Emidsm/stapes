#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "stapes/hardware/robot_interface.hpp"
#include "stapes/hardware/safety_interface.hpp"

namespace stapes::faults {

enum class FaultKind : std::uint8_t {
  none,
  frozen_motor_encoder,  ///< One motor encoder repeats its last value. Its timestamp stays fresh.
  frozen_link_encoder,   ///< The same, for one link encoder on the safety channel.
  frozen_tracker,        ///< The tracker repeats its last pose. Its timestamp stays fresh.
  stuck_actuator,        ///< One motor keeps its last torque and ignores new commands.
  comms_delay,           ///< Control readings and commands both arrive late.
};

struct Fault {
  FaultKind kind{FaultKind::none};
  std::int64_t start_tick{0};
  std::size_t joint{0};  ///< For the faults that affect one joint.
  int delay_ticks{0};    ///< For `comms_delay`.
};

/// Sits between the software and the robot, and implements the same interfaces.
///
/// Neither side can tell it is there, so the code under test runs unmodified, and the plant
/// needs no knowledge of faults. Delays are held in fixed buffers in simulated time, with no
/// threads and no allocation.
///
/// Brakes always pass straight through: they are the path a stop must not depend on.
class FaultInjector final : public hardware::RobotInterface, public hardware::SafetyInterface {
 public:
  /// The longest `comms_delay` that can be represented.
  static constexpr int kMaxDelayTicks = 255;

  /// @param robot and @param safety are the real interfaces. They must outlive this object.
  /// @throws std::invalid_argument if the joint or the delay is out of range.
  FaultInjector(hardware::RobotInterface& robot, hardware::SafetyInterface& safety,
                const Fault& fault);

  /// Call once per control period, after the robot has advanced, to take in its new readings.
  void update() noexcept;

  [[nodiscard]] hardware::SensorSample read_sensors() const override { return control_sample_; }
  void command_torques(const kinematics::JointVector& torques_nm) override;

  [[nodiscard]] hardware::SafetySample read_safety_sensors() const override {
    return safety_sample_;
  }
  void engage_brakes() override { safety_->engage_brakes(); }
  void release_brakes() override { safety_->release_brakes(); }

  /// Moves the start of the fault, for faults started by an operator rather than at a planned
  /// tick. A start in the past takes effect at the next `update`.
  void start_at(std::int64_t start_tick) noexcept { fault_.start_tick = start_tick; }

  [[nodiscard]] bool active() const noexcept {
    return fault_.kind != FaultKind::none && now_ticks_ >= fault_.start_tick;
  }

 private:
  static constexpr std::size_t kBufferSize = kMaxDelayTicks + 1;
  [[nodiscard]] static std::size_t slot(std::int64_t tick) noexcept;
  [[nodiscard]] bool is_active(FaultKind kind) const noexcept;

  // Not owned, and never null: the constructor takes them by reference.
  hardware::RobotInterface* robot_;
  hardware::SafetyInterface* safety_;
  Fault fault_;
  std::int64_t now_ticks_{0};
  hardware::SensorSample control_sample_{};
  hardware::SafetySample safety_sample_{};
  kinematics::JointVector last_torques_{kinematics::JointVector::Zero()};
  std::array<hardware::SensorSample, kBufferSize> reading_history_{};
  std::array<kinematics::JointVector, kBufferSize> command_history_{};
};

}  // namespace stapes::faults
