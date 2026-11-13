#pragma once

#include <cstdint>
#include <optional>

#include "stapes/base/state_hash.hpp"
#include "stapes/control/joint_controller.hpp"
#include "stapes/faults/fault_injector.hpp"
#include "stapes/plant/simulated_arm.hpp"
#include "stapes/safety/safety_monitor.hpp"
#include "stapes/task/keyhole.hpp"

namespace stapes::sim {

struct KeyholeScenario {
  plant::SimulatedArmConfig arm;
  control::JointGains gains{};
  control::GearboxEstimates gearboxes{};
  task::KeyholeTrajectory trajectory;
  double feed_speed_m_s{0.0};
  double feed_acceleration_m_s2{0.0};
  safety::SafetyConfig safety;
  faults::Fault fault;

  /// The reference arm drilling a 10 mm tunnel, with no fault. The anatomy is illustrative: the
  /// tunnel is a reachable straight line, tilted out of the arm's plane so that every joint has
  /// to move along it, which leaves a fault nowhere to hide.
  [[nodiscard]] static KeyholeScenario reference(std::uint64_t seed);
};

/// A complete run: robot, fault injector, controller and safety monitor, in one tick order.
///
/// The controller reads and commands through the fault injector. The monitor checks each period
/// before the controller runs, so a stop takes effect in the same period it is detected.
class KeyholeSimulation {
 public:
  /// @throws std::invalid_argument if the plan is not reachable from the arm's initial
  ///         configuration. Nothing has moved at that point.
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
  [[nodiscard]] double planned_depth_m() const noexcept { return planned_depth_m_; }
  [[nodiscard]] double feed_duration_s() const noexcept { return profile_.duration_s(); }
  [[nodiscard]] std::int64_t ticks() const noexcept { return arm_.clock().ticks(); }

  /// The tick at which the monitor stopped the robot, if it has.
  [[nodiscard]] std::optional<std::int64_t> stop_tick() const noexcept { return stop_tick_; }

  /// True tool pose, for measuring results. Nothing in the running system reads this.
  [[nodiscard]] math::Pose true_tool_pose() const noexcept;

  void add_to_hash(base::StateHash& hash) const noexcept;

 private:
  plant::SimulatedArm arm_;
  faults::FaultInjector injector_;
  control::JointController controller_;
  safety::SafetyMonitor monitor_;
  task::KeyholeTrajectory trajectory_;
  task::FeedProfile profile_;
  kinematics::ArmModel plan_model_;
  double planned_depth_m_{0.0};
  kinematics::JointVector target_;
  std::optional<std::int64_t> stop_tick_;
};

}  // namespace stapes::sim
