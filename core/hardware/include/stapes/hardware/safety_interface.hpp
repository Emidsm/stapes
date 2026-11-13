#pragma once

#include <cstdint>

#include "stapes/kinematics/arm_model.hpp"
#include "stapes/math/rotation.hpp"
#include "stapes/math/wrench.hpp"

namespace stapes::hardware {

/// The readings that reach the safety monitor without passing through the control path.
struct SafetySample {
  kinematics::JointVector link_positions_rad;  ///< From encoders on the output of each gearbox.
  math::Pose tracked_tool_pose;                ///< From an optical tracker, in world coordinates.
  math::Wrench tool_wrench;                    ///< In the tool frame.
  std::int64_t stamp_ticks{0};                 ///< Control tick at which this was sampled.
};

/// The safety channel: its own sensors, and a way to stop that needs no cooperation from the
/// controller.
///
/// The force sensor is the one sensor shared with the control path: the robot carries only
/// one, so a fault in it affects both.
class SafetyInterface {
 public:
  SafetyInterface() = default;
  SafetyInterface(const SafetyInterface&) = default;
  SafetyInterface(SafetyInterface&&) = default;
  SafetyInterface& operator=(const SafetyInterface&) = default;
  SafetyInterface& operator=(SafetyInterface&&) = default;
  virtual ~SafetyInterface() = default;

  [[nodiscard]] virtual SafetySample read_safety_sensors() const = 0;

  /// Removes motor torque and applies the brakes. Torque commands have no effect until
  /// `release_brakes`.
  virtual void engage_brakes() = 0;
  virtual void release_brakes() = 0;
};

}  // namespace stapes::hardware
