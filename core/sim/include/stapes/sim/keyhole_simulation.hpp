#pragma once

#include <cstdint>
#include <optional>

#include "stapes/base/state_hash.hpp"
#include "stapes/control/joint_controller.hpp"
#include "stapes/faults/fault_injector.hpp"
#include "stapes/plant/simulated_arm.hpp"
#include "stapes/safety/safety_monitor.hpp"
#include "stapes/task/joint_move.hpp"
#include "stapes/task/keyhole.hpp"

namespace stapes::sim {

/// How the arm gets to the tunnel when it does not start there.
///
/// The arm starts at rest at `rest_q`, moves in joint space to a point `standoff_m` before the
/// entry on the tunnel's line, waits `settle_s` there, and then feeds straight in. Arriving on
/// the line from outside the monitor's entry guard is what lets the monitor hold the tip to the
/// line before it reaches the entry, with no phase signal from the controller.
struct Approach {
  kinematics::JointVector rest_q{kinematics::JointVector::Zero()};
  double move_s{0.0};      ///< Duration of the joint-space move.
  double settle_s{0.0};    ///< Wait on the line before the feed starts.
  double standoff_m{0.0};  ///< How far before the entry the move ends.
};

struct KeyholeScenario {
  plant::SimulatedArmConfig arm;
  control::JointGains gains{};
  control::GearboxEstimates gearboxes{};
  task::KeyholeTrajectory trajectory;
  double feed_speed_m_s{0.0};
  double feed_acceleration_m_s2{0.0};
  safety::SafetyConfig safety;
  faults::Fault fault;
  /// Empty: the arm starts at the entry, already on the line.
  std::optional<Approach> approach;

  /// The reference arm drilling a 10 mm tunnel, with no fault. The anatomy is illustrative: the
  /// tunnel is a reachable straight line, tilted out of the arm's plane so that every joint has
  /// to move along it, which leaves a fault nowhere to hide.
  [[nodiscard]] static KeyholeScenario reference(std::uint64_t seed);
};

/// The same scenario with every monitor limit too wide to ever trip. The monitor stays in the
/// loop but cannot act.
///
/// This exists to measure the baseline: what a fault does when nothing contains it. Without
/// that number, a fault campaign would also pass if its faults were simply harmless.
[[nodiscard]] KeyholeScenario without_monitor_limits(KeyholeScenario scenario);

/// The same scenario, starting from a rest pose 12 cm back along the tool and 8 cm up instead
/// of at the entry. The move is slow enough to stay inside the monitor's speed and tracking
/// limits, which are the same throughout the run.
///
/// @throws std::invalid_argument if the rest pose or the line to the target is not reachable.
[[nodiscard]] KeyholeScenario with_approach(KeyholeScenario scenario);

/// A complete run: robot, fault injector, controller and safety monitor, in one tick order.
///
/// The controller reads and commands through the fault injector. The monitor checks each period
/// before the controller runs, so a stop takes effect in the same period it is detected.
class KeyholeSimulation {
 public:
  /// Everything worked out before the arm is built. Defined where it is used.
  struct Plan;

  /// @throws std::invalid_argument if the plan is not reachable from the arm's initial
  ///         configuration, or if an approach would bring the tip inside the monitor's entry
  ///         guard before it is on the line. Nothing has moved at that point.
  explicit KeyholeSimulation(const KeyholeScenario& scenario);

  KeyholeSimulation(const KeyholeSimulation&) = delete;
  KeyholeSimulation& operator=(const KeyholeSimulation&) = delete;
  KeyholeSimulation(KeyholeSimulation&&) = delete;
  KeyholeSimulation& operator=(KeyholeSimulation&&) = delete;
  ~KeyholeSimulation() = default;

  /// Advances one control period.
  void step() noexcept;

  void set_external_wrench(const math::Wrench& world_wrench) noexcept {
    arm_.set_external_wrench(world_wrench);
  }

  [[nodiscard]] const plant::SimulatedArm& arm() const noexcept { return arm_; }
  [[nodiscard]] const safety::SafetyMonitor& monitor() const noexcept { return monitor_; }
  [[nodiscard]] const task::KeyholeTrajectory& trajectory() const noexcept { return trajectory_; }
  [[nodiscard]] const kinematics::JointVector& target() const noexcept { return target_; }
  /// Planned depth along the tunnel. Negative before the entry, during an approach.
  [[nodiscard]] double planned_depth_m() const noexcept { return planned_depth_m_; }
  /// When the straight feed starts: zero without an approach.
  [[nodiscard]] double feed_start_s() const noexcept { return feed_start_s_; }
  /// Duration of the straight feed, including the standoff before the entry.
  [[nodiscard]] double feed_duration_s() const noexcept { return profile_.duration_s(); }

  /// Whether the arm is still on its way to the line. For labelling results: the monitor
  /// cannot ask this, and does not need to.
  [[nodiscard]] bool approaching() const noexcept;
  [[nodiscard]] std::int64_t ticks() const noexcept { return arm_.clock().ticks(); }

  /// Whether the scenario's fault has started. For labelling results: nothing in the running
  /// system can ask this.
  [[nodiscard]] bool fault_active() const noexcept { return injector_.active(); }

  /// The tick at which the monitor stopped the robot, if it has.
  [[nodiscard]] std::optional<std::int64_t> stop_tick() const noexcept { return stop_tick_; }

  /// True tool pose, for measuring results. Nothing in the running system reads this.
  [[nodiscard]] math::Pose true_tool_pose() const noexcept;

  void add_to_hash(base::StateHash& hash) const noexcept;

 private:
  KeyholeSimulation(const KeyholeScenario& scenario, const Plan& plan);

  plant::SimulatedArm arm_;
  faults::FaultInjector injector_;
  control::JointController controller_;
  safety::SafetyMonitor monitor_;
  task::KeyholeTrajectory trajectory_;
  task::KeyholeTrajectory feed_line_;  ///< From the standoff point to the target.
  task::FeedProfile profile_;
  std::optional<task::JointMove> move_;
  double standoff_m_{0.0};
  double feed_start_s_{0.0};
  kinematics::ArmModel plan_model_;
  double planned_depth_m_;
  kinematics::JointVector target_;
  std::optional<std::int64_t> stop_tick_;
};

}  // namespace stapes::sim
