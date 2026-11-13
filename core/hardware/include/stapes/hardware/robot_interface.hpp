#pragma once

#include <cstdint>

#include "stapes/kinematics/arm_model.hpp"
#include "stapes/math/wrench.hpp"

namespace stapes::hardware {

/// One consistent set of sensor readings.
struct SensorSample {
  kinematics::JointVector joint_positions_rad;
  math::Wrench tool_wrench;     ///< In the tool frame.
  std::int64_t stamp_ticks{0};  ///< Control tick at which this was sampled.
};

/// Everything the control software may know about the robot.
///
/// This is the boundary between software and hardware. A real robot would implement
/// this same interface, so code on this side cannot depend on being simulated. It only
/// sees measurements, never the true state.
class RobotInterface {
 public:
  RobotInterface() = default;
  RobotInterface(const RobotInterface&) = default;
  RobotInterface(RobotInterface&&) = default;
  RobotInterface& operator=(const RobotInterface&) = default;
  RobotInterface& operator=(RobotInterface&&) = default;
  virtual ~RobotInterface() = default;

  [[nodiscard]] virtual SensorSample read_sensors() const = 0;
  virtual void command_torques(const kinematics::JointVector& torques_nm) = 0;
};

}  // namespace stapes::hardware
