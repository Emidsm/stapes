#include "stapes/sim/keyhole_simulation.hpp"

#include <chrono>
#include <stdexcept>

#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"

#include "planning.hpp"

namespace stapes::sim {

namespace {

double seconds(base::SimDuration duration) {
  return std::chrono::duration<double>(duration).count();
}

// The straight line the feed follows: the tunnel, extended back to the standoff point.
task::KeyholeTrajectory feed_line_of(const KeyholeScenario& scenario) {
  return scenario.approach ? detail::feed_line(scenario.trajectory, scenario.approach->standoff_m)
                           : scenario.trajectory;
}

// The configuration at the start of the feed.
kinematics::JointVector feed_start_q(const KeyholeScenario& scenario,
                                     const task::KeyholeTrajectory& feed_line,
                                     const kinematics::JointVector& seed) {
  return detail::line_start_q(scenario.arm.model.arm(), feed_line, seed);
}

}  // namespace

struct KeyholeSimulation::Plan {
  plant::SimulatedArmConfig arm;
  task::KeyholeTrajectory feed_line;
  std::optional<task::JointMove> move;
};

namespace {

KeyholeSimulation::Plan make_plan(const KeyholeScenario& scenario) {
  const task::KeyholeTrajectory feed_line = feed_line_of(scenario);
  const kinematics::JointVector on_line = feed_start_q(scenario, feed_line, scenario.arm.initial_q);
  plant::SimulatedArmConfig config = scenario.arm;
  if (!scenario.approach) {
    config.initial_q = on_line;
    return {config, feed_line, std::nullopt};
  }

  const Approach& approach = *scenario.approach;
  const task::JointMove move(approach.rest_q, on_line, approach.move_s);
  // The monitor holds the tip to the line once it is inside the guard. A move that cut inside
  // the guard before reaching the line would be stopped, so it is refused before it starts.
  if (detail::move_enters_guard(scenario.arm.model.arm(), move, scenario.trajectory,
                                scenario.safety.corridor.entry_guard_m)) {
    throw std::invalid_argument("the approach enters the entry guard off the line");
  }
  config.initial_q = approach.rest_q;
  return {config, feed_line, move};
}

}  // namespace

KeyholeScenario KeyholeScenario::reference(std::uint64_t seed) {
  constexpr double kTunnelLengthM = 10e-3;
  constexpr double kFeedSpeedMS = 5e-3;
  constexpr double kFeedAccelerationMS2 = 25e-3;
  constexpr double kCorridorRadiusM = 0.3e-3;
  constexpr double kOvershootMarginM = 0.3e-3;
  constexpr double kEntryGuardM = 2e-3;

  plant::SimulatedArmConfig arm = plant::SimulatedArmConfig::reference(seed);
  arm.initial_q << 0.5, -0.3, 0.6, 0.4, 0.8, 0.3;
  const math::Pose start = kinematics::forward_kinematics(arm.model.arm(), arm.initial_q);
  const Eigen::Vector3d entry = start.translation();
  // Roughly along the tool, but not exactly, so that the wrist has to rotate too.
  const Eigen::Vector3d direction =
      (start.linear().col(0) + Eigen::Vector3d(0.0, 0.3, 0.1)).normalized();
  const Eigen::Vector3d target = entry + (kTunnelLengthM * direction);
  return KeyholeScenario{
      arm,
      control::reference_gains(),
      control::reference_gearbox_estimates(),
      task::KeyholeTrajectory(entry, target, start),
      kFeedSpeedMS,
      kFeedAccelerationMS2,
      safety::reference_safety_config(
          safety::Corridor{entry, target, kCorridorRadiusM, kOvershootMarginM, kEntryGuardM}),
      faults::Fault{},
      std::nullopt,
  };
}

KeyholeScenario with_approach(KeyholeScenario scenario) {
  constexpr double kRetreatM = 0.12;
  constexpr double kLiftM = 0.08;
  // Half the monitor's speed limit, which holds during the approach as much as while drilling.
  constexpr double kPeakSpeedFraction = 0.5;
  constexpr double kSettleS = 0.5;
  constexpr double kStandoffM = 5e-3;

  const task::KeyholeTrajectory& tunnel = scenario.trajectory;
  scenario.approach = Approach{kinematics::JointVector::Zero(), 1.0, kSettleS, kStandoffM};
  const kinematics::JointVector on_line =
      feed_start_q(scenario, feed_line_of(scenario), scenario.arm.initial_q);

  // At rest: pulled back along the tunnel and lifted, with the tool pointing the same way.
  math::Pose rest = tunnel.pose_at(0.0);
  rest.translation() +=
      (-(kStandoffM + kRetreatM) * tunnel.direction()) + Eigen::Vector3d(0.0, 0.0, kLiftM);
  const kinematics::IkResult solved = kinematics::solve_ik(scenario.arm.model.arm(), rest, on_line);
  if (!solved.converged()) {
    throw std::invalid_argument("the rest pose is not reachable");
  }
  scenario.approach->rest_q = solved.q;
  scenario.approach->move_s =
      detail::move_duration_s(solved.q, on_line, scenario.safety, kPeakSpeedFraction);
  return scenario;
}

KeyholeScenario without_monitor_limits(KeyholeScenario scenario) {
  constexpr double kNever = 1e9;
  constexpr std::int64_t kNeverTicks = 1000000;
  safety::SafetyConfig& limits = scenario.safety;
  kinematics::ArmModel::Joints joints = limits.arm.joints();
  for (kinematics::RevoluteJoint& joint : joints) {
    joint.lower_rad = -kNever;
    joint.upper_rad = kNever;
  }
  limits.arm = kinematics::ArmModel(joints, limits.arm.tool());
  limits.joint_limit_margin_rad = 0.0;
  limits.max_velocity_rad_s.setConstant(kNever);
  limits.max_tool_force_n = kNever;
  limits.corridor.radius_m = kNever;
  limits.corridor.overshoot_margin_m = kNever;
  limits.corridor.entry_guard_m = kNever;
  limits.max_tracker_disagreement_m = kNever;
  limits.max_encoder_disagreement_rad.setConstant(kNever);
  limits.max_tracking_error_rad.setConstant(kNever);
  limits.max_sample_age_ticks = kNeverTicks;
  return scenario;
}

KeyholeSimulation::KeyholeSimulation(const KeyholeScenario& scenario)
    : KeyholeSimulation(scenario, make_plan(scenario)) {}

KeyholeSimulation::KeyholeSimulation(const KeyholeScenario& scenario, const Plan& plan)
    : arm_(plan.arm),
      injector_(arm_, arm_, scenario.fault),
      controller_(scenario.arm.model, scenario.gains, scenario.gearboxes,
                  seconds(scenario.arm.control_period)),
      monitor_(scenario.safety),
      trajectory_(scenario.trajectory),
      feed_line_(plan.feed_line),
      profile_(plan.feed_line.length_m(), scenario.feed_speed_m_s, scenario.feed_acceleration_m_s2),
      move_(plan.move),
      standoff_m_(scenario.approach ? scenario.approach->standoff_m : 0.0),
      feed_start_s_(scenario.approach ? scenario.approach->move_s + scenario.approach->settle_s
                                      : 0.0),
      plan_model_(scenario.arm.model.arm()),
      planned_depth_m_(-standoff_m_),
      target_(arm_.state().q) {}

bool KeyholeSimulation::approaching() const noexcept {
  return arm_.clock().now_seconds() < feed_start_s_;
}

void KeyholeSimulation::step() noexcept {
  if (stop_tick_.has_value()) {
    arm_.tick();
    injector_.update();
    return;
  }

  const double now_s = arm_.clock().now_seconds();
  if (move_.has_value() && now_s < feed_start_s_) {
    // On the way to the line, and then waiting on it. The move holds its end after `move_s`.
    target_ = move_->at(now_s);
  } else {
    const double feed_depth_m = profile_.depth_at(now_s - feed_start_s_);
    planned_depth_m_ = feed_depth_m - standoff_m_;
    const kinematics::IkResult solved =
        kinematics::solve_ik(plan_model_, feed_line_.pose_at(feed_depth_m), target_);
    // Checked before the run for every 0.5 mm of the line, so a miss here is unexpected.
    // Holding the last target is the conservative response, and the monitor sees the tracking
    // error.
    if (solved.converged()) {
      target_ = solved.q;
    }
  }

  const hardware::SensorSample control_reading = injector_.read_sensors();
  const safety::SafetyInput input{injector_.read_safety_sensors(), control_reading, target_,
                                  arm_.clock().ticks()};
  if (monitor_.check(input) != safety::StopCause::none) {
    injector_.engage_brakes();
    stop_tick_ = arm_.clock().ticks();
  } else {
    injector_.command_torques(controller_.update(target_, control_reading.joint_positions_rad));
  }

  arm_.tick();
  injector_.update();
}

math::Pose KeyholeSimulation::true_tool_pose() const noexcept {
  return kinematics::forward_kinematics(plan_model_, arm_.state().q);
}

void KeyholeSimulation::add_to_hash(base::StateHash& hash) const noexcept {
  arm_.add_to_hash(hash);
  hash.add(static_cast<std::uint64_t>(monitor_.cause()));
  hash.add(static_cast<std::uint64_t>(stop_tick_.value_or(-1)));
}

}  // namespace stapes::sim
