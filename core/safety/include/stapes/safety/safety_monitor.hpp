#pragma once

#include <cstdint>

#include <Eigen/Geometry>

#include "stapes/hardware/robot_interface.hpp"
#include "stapes/hardware/safety_interface.hpp"
#include "stapes/kinematics/arm_model.hpp"

namespace stapes::safety {

/// Why the monitor stopped the robot. Checks run in this order, and the first one that fails
/// decides the cause.
enum class StopCause : std::uint8_t {
  none,
  non_finite_input,      ///< A reading or the target contains NaN or infinity.
  stale_data,            ///< A sample is too old, or stamped in the future.
  joint_limit,           ///< A link is outside its safe range.
  joint_velocity,        ///< A link moves too fast, from successive link readings.
  tool_force,            ///< The force on the tool is too high.
  corridor,              ///< The tool has left the planned tunnel.
  tracker_disagreement,  ///< The tracker and the link encoders disagree about the tip.
  encoder_disagreement,  ///< A motor encoder and its link encoder disagree.
  tracking_error,        ///< A link is too far from where the controller was told to put it.
};

/// The planned tunnel, as a straight line with an allowed radius.
struct Corridor {
  Eigen::Vector3d entry_m;
  Eigen::Vector3d target_m;
  double radius_m;            ///< Allowed distance from the line, for the tip and at the entry.
  double overshoot_margin_m;  ///< Allowed travel past the target.
};

struct SafetyConfig {
  kinematics::ArmModel arm;  ///< The monitor's own copy of the arm geometry.
  Corridor corridor;
  double joint_limit_margin_rad;  ///< Stop this far before each limit in `arm`.
  double period_s;                ///< Time between calls to `check`.
  kinematics::JointVector max_velocity_rad_s;
  double max_tool_force_n;
  double max_tracker_disagreement_m;
  kinematics::JointVector max_encoder_disagreement_rad;
  kinematics::JointVector max_tracking_error_rad;
  std::int64_t max_sample_age_ticks;
  int debounce_checks;  ///< How many consecutive checks a noisy condition must fail.
};

/// Everything the monitor sees in one period. These are copies, so it shares no state with the
/// controller.
struct SafetyInput {
  hardware::SafetySample safety;   ///< The monitor's own channel.
  hardware::SensorSample control;  ///< What the controller is reading.
  kinematics::JointVector target;  ///< Where the controller has been told to put the links.
  std::int64_t now_ticks;
};

/// Watches the robot and decides when it must stop. It is independent of the controller: it
/// has its own sensors, and it does not link the control library.
///
/// A stop is latched. Once `check` has returned a cause, it keeps returning that same cause,
/// whatever later input says, until `try_reset` succeeds.
///
/// Neither `check` nor `try_reset` allocates, throws or reads a clock.
class SafetyMonitor {
 public:
  /// @throws std::invalid_argument unless every limit is positive and finite (the joint margin
  ///         and overshoot margin may be zero), the corridor end points are finite and distinct,
  ///         and `debounce_checks` is at least 1.
  explicit SafetyMonitor(const SafetyConfig& config);

  /// Runs every check on one period's input, and returns the latched cause.
  ///
  /// These fail at once: `non_finite_input`, `stale_data`, `joint_limit`, `joint_velocity`.
  /// The first call has no earlier reading, so it does not check velocity.
  ///
  /// These fail only after `debounce_checks` consecutive violations, because they rest on noisy
  /// sensors: `tool_force`, `corridor`, `tracker_disagreement`, `encoder_disagreement`,
  /// `tracking_error`. Each keeps its own count, and a clean check resets that count.
  [[nodiscard]] StopCause check(const SafetyInput& input) noexcept;

  /// Clears a latched stop if `input` passes every check right now, ignoring debounce and
  /// velocity. On success it also forgets all counts and the earlier reading, as if newly built.
  ///
  /// @return whether the monitor is now clear to run. It stays stopped otherwise, keeping its
  ///         original cause. Calling this while not stopped changes nothing and returns true.
  [[nodiscard]] bool try_reset(const SafetyInput& input) noexcept;

  [[nodiscard]] bool stopped() const noexcept { return cause_ != StopCause::none; }
  [[nodiscard]] StopCause cause() const noexcept { return cause_; }

 private:
  StopCause cause_{StopCause::none};
};

/// A short name for logs and reports.
[[nodiscard]] const char* to_string(StopCause cause) noexcept;

/// Limits for the reference arm following `corridor`.
[[nodiscard]] SafetyConfig reference_safety_config(const Corridor& corridor);

}  // namespace stapes::safety
