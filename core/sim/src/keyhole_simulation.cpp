#include "stapes/sim/keyhole_simulation.hpp"

#include <chrono>
#include <stdexcept>

#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"

namespace stapes::sim {

namespace {

constexpr double kPlanStepM = 0.5e-3;

double seconds(base::SimDuration duration) {
  return std::chrono::duration<double>(duration).count();
}

// Runs before anything is built, and starts the arm on the plan instead of beside it.
plant::SimulatedArmConfig on_plan(const KeyholeScenario& scenario) {
  plant::SimulatedArmConfig config = scenario.arm;
  kinematics::JointVector start;
  if (!task::plan_is_reachable(config.model.arm(), scenario.trajectory, config.initial_q,
                               kPlanStepM, start)) {
    throw std::invalid_argument("the planned tunnel is not reachable");
  }
  config.initial_q = start;
  return config;
}

}  // namespace

KeyholeScenario KeyholeScenario::reference(std::uint64_t seed) {
  constexpr double kTunnelLengthM = 10e-3;
  constexpr double kFeedSpeedMS = 5e-3;
  constexpr double kFeedAccelerationMS2 = 25e-3;
  constexpr double kCorridorRadiusM = 0.3e-3;
  constexpr double kOvershootMarginM = 0.3e-3;

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
          safety::Corridor{entry, target, kCorridorRadiusM, kOvershootMarginM}),
      faults::Fault{},
  };
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
  limits.max_tracker_disagreement_m = kNever;
  limits.max_encoder_disagreement_rad.setConstant(kNever);
  limits.max_tracking_error_rad.setConstant(kNever);
  limits.max_sample_age_ticks = kNeverTicks;
  return scenario;
}

KeyholeSimulation::KeyholeSimulation(const KeyholeScenario& scenario)
    : arm_(on_plan(scenario)),
      injector_(arm_, arm_, scenario.fault),
      controller_(scenario.arm.model, scenario.gains, scenario.gearboxes,
                  seconds(scenario.arm.control_period)),
      monitor_(scenario.safety),
      trajectory_(scenario.trajectory),
      profile_(scenario.trajectory.length_m(), scenario.feed_speed_m_s,
               scenario.feed_acceleration_m_s2),
      plan_model_(scenario.arm.model.arm()),
      target_(arm_.state().q) {}

void KeyholeSimulation::step() noexcept {
  if (stop_tick_.has_value()) {
    arm_.tick();
    injector_.update();
    return;
  }

  planned_depth_m_ = profile_.depth_at(arm_.clock().now_seconds());
  const kinematics::IkResult solved =
      kinematics::solve_ik(plan_model_, trajectory_.pose_at(planned_depth_m_), target_);
  // Checked before the run for every 0.5 mm of the tunnel, so a miss here is unexpected. Holding
  // the last target is the conservative response, and the monitor sees the tracking error.
  if (solved.converged()) {
    target_ = solved.q;
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
