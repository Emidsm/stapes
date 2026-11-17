#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Core>

#include "stapes/base/state_hash.hpp"
#include "stapes/control/joint_controller.hpp"
#include "stapes/faults/fault_injector.hpp"
#include "stapes/plant/simulated_arm.hpp"
#include "stapes/safety/safety_monitor.hpp"
#include "stapes/sim/keyhole_simulation.hpp"
#include "stapes/task/joint_move.hpp"
#include "stapes/task/keyhole.hpp"

namespace stapes::sim {

/// What an operator asks for in one control period, from a gamepad or from a recording.
///
/// The buttons are levels, as a gamepad reports them. The simulation acts when one goes from
/// released to pressed, so holding a button down does the thing once.
struct OperatorInput {
  /// Tip velocity in the world frame, in free mode. Longer than the speed limit is cut to it.
  Eigen::Vector3d tip_velocity_m_s{Eigen::Vector3d::Zero()};
  /// On the line: the feed as a fraction of the planned feed speed, from -1 (back out) to 1.
  double feed{0.0};
  /// Free: move onto the planned line. On the line, at the standoff: let go of it.
  bool align{false};
  /// Start the scenario's fault now.
  bool fault{false};
  /// Free: switch to moving one joint at a time. Joint: switch back.
  bool joint_mode{false};
  /// In joint mode, the joint to move, from 0 at the base. Out of range moves none.
  int joint{0};
  /// In joint mode, that joint's speed as a fraction of its jog speed limit, from -1 to 1.
  double joint_rate{0.0};
};

/// What the operator is doing with the arm.
enum class JogMode : std::uint8_t {
  free,      ///< The operator moves the tip. The tool keeps its direction.
  aligning,  ///< A planned move in joint space onto the line, at the standoff.
  on_line,   ///< Held on the line. The operator sets only how fast the tip goes along it.
  joint,     ///< The operator turns one joint at a time, as on a robot's teach pendant.
};

/// Why the last press of `align` or `joint_mode` did nothing.
enum class AlignRefusal : std::uint8_t {
  none,
  still_moving,     ///< The arm is still moving: switching starts from rest.
  enters_guard,     ///< The move to the line would cut inside the entry guard off the line.
  not_at_standoff,  ///< Letting go of the line is allowed only back at the standoff.
};

/// The same robot, controller and safety monitor as `KeyholeSimulation`, with an operator in
/// place of the plan: a hands-on mode for demonstrating the monitor live.
///
/// The operator can move the tip freely, or have the arm align itself with the planned line and
/// then control the feed along it. Nothing the operator does reaches the monitor, which sees
/// the same sensors and applies the same limits as in a planned run. Moving the tip into the
/// bone off the line is the operator's to try and the monitor's to stop.
///
/// Every run is a function of the scenario and the sequence of inputs, so a recorded session
/// replays exactly.
class JogSimulation {
 public:
  /// The fastest the operator can move the tip in free mode. Near the rest pose, slowing for the
  /// joints alone would allow 16 to 31 mm/s depending on the direction.
  static constexpr double kMaxTipSpeedMS = 0.025;
  /// How fast the commanded tip speed may change. The joint controller lags an accelerating
  /// target, and the monitor stops the arm when the lag passes its tracking limit. At 0.1 m/s^2
  /// a stick pushed hard does that within 0.15 s, and at 0.05 m/s^2 reversing the stick does
  /// it, when the gearboxes cross their backlash. This value keeps the lag under half the limit.
  static constexpr double kMaxTipAccelerationMS2 = 0.025;
  /// Free mode slows the tip wherever its speed would take a joint past this fraction of the
  /// monitor's joint speed limit, as it does near a singularity.
  static constexpr double kJointSpeedFraction = 0.5;
  /// In joint mode, how fast a joint's commanded speed may change, in rad/s^2. Like the tip's
  /// limit, it keeps the controller's lag under half the monitor's tracking limit.
  static constexpr double kMaxJointAccelerationRadS2 = 0.05;
  /// In joint mode, each joint stops this far inside the monitor's own margin before its limit,
  /// slowing down in time: a soft limit, so that jogging to the end of a joint is not a stop.
  static constexpr double kSoftLimitMarginRad = 0.02;

  /// Starts at rest at the scenario's approach pose. The scenario's fault is armed, not started:
  /// its kind, joint and delay apply, its start tick does not.
  ///
  /// @throws std::invalid_argument if the scenario has no approach, or its line is not
  ///         reachable.
  explicit JogSimulation(const KeyholeScenario& scenario);

  JogSimulation(const JogSimulation&) = delete;
  JogSimulation& operator=(const JogSimulation&) = delete;
  JogSimulation(JogSimulation&&) = delete;
  JogSimulation& operator=(JogSimulation&&) = delete;
  ~JogSimulation() = default;

  /// Advances one control period with the operator's input for that period.
  void step(const OperatorInput& input) noexcept;

  [[nodiscard]] JogMode mode() const noexcept { return mode_; }
  [[nodiscard]] AlignRefusal align_refusal() const noexcept { return refusal_; }
  [[nodiscard]] const plant::SimulatedArm& arm() const noexcept { return arm_; }
  [[nodiscard]] const safety::SafetyMonitor& monitor() const noexcept { return monitor_; }
  [[nodiscard]] const task::KeyholeTrajectory& trajectory() const noexcept { return trajectory_; }
  [[nodiscard]] const kinematics::JointVector& target() const noexcept { return target_; }
  /// How far the planned tip is along the tunnel while on the line. Negative before the entry.
  [[nodiscard]] double planned_depth_m() const noexcept { return line_depth_m_ - standoff_m_; }
  [[nodiscard]] std::int64_t ticks() const noexcept { return arm_.clock().ticks(); }
  [[nodiscard]] bool fault_active() const noexcept { return injector_.active(); }
  [[nodiscard]] std::optional<std::int64_t> fault_start_tick() const noexcept {
    return fault_start_tick_;
  }
  [[nodiscard]] std::optional<std::int64_t> stop_tick() const noexcept { return stop_tick_; }

  /// True tool pose, for display and measurement. Nothing in the running system reads it.
  [[nodiscard]] math::Pose true_tool_pose() const noexcept;

  void add_to_hash(base::StateHash& hash) const noexcept;

 private:
  void jog(const Eigen::Vector3d& velocity_m_s) noexcept;
  void align() noexcept;
  void follow_line(double feed) noexcept;
  void turn_joint(int joint, double rate) noexcept;
  [[nodiscard]] bool arm_moving() const noexcept;

  plant::SimulatedArm arm_;
  faults::FaultInjector injector_;
  control::JointController controller_;
  safety::SafetyMonitor monitor_;
  task::KeyholeTrajectory trajectory_;
  task::KeyholeTrajectory line_;  ///< From the standoff point to the target.
  kinematics::ArmModel plan_model_;
  safety::SafetyConfig limits_;  ///< For planning moves: the monitor keeps its own copy.
  double standoff_m_;
  double feed_speed_m_s_;
  double feed_acceleration_m_s2_;
  double period_s_;
  kinematics::JointVector line_start_q_;
  math::Pose tip_target_;  ///< Free mode: where the operator has put the tip.
  Eigen::Vector3d tip_velocity_m_s_{Eigen::Vector3d::Zero()};
  std::optional<task::JointMove> move_;
  double move_start_s_{0.0};
  double line_depth_m_{0.0};  ///< Along `line_`, from the standoff point.
  double line_speed_m_s_{0.0};
  kinematics::JointVector joint_velocity_rad_s_{kinematics::JointVector::Zero()};
  kinematics::JointVector target_;
  JogMode mode_{JogMode::free};
  AlignRefusal refusal_{AlignRefusal::none};
  OperatorInput previous_{};
  std::optional<std::int64_t> fault_start_tick_;
  std::optional<std::int64_t> stop_tick_;
};

/// A short name for logs and the viewer.
[[nodiscard]] const char* to_string(JogMode mode) noexcept;
[[nodiscard]] const char* to_string(AlignRefusal refusal) noexcept;

}  // namespace stapes::sim
