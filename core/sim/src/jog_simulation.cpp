#include "stapes/sim/jog_simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"

#include "planning.hpp"

namespace stapes::sim {

namespace {

// The move onto the line runs its fastest joint at this fraction of that joint's limit, as the
// planned approach does.
constexpr double kAlignSpeedFraction = 0.5;
// A move onto the line takes at least this long, even from very close to it.
constexpr double kMinAlignS = 0.2;

double seconds(base::SimDuration duration) {
  return std::chrono::duration<double>(duration).count();
}

const Approach& approach_of(const KeyholeScenario& scenario) {
  if (!scenario.approach) {
    throw std::invalid_argument("jogging starts from a rest pose: the scenario needs an approach");
  }
  return *scenario.approach;
}

plant::SimulatedArmConfig at_rest(const KeyholeScenario& scenario) {
  plant::SimulatedArmConfig config = scenario.arm;
  config.initial_q = approach_of(scenario).rest_q;
  return config;
}

// The scenario's fault, armed but not started: the operator decides when.
faults::Fault armed(faults::Fault fault) {
  fault.start_tick = std::numeric_limits<std::int64_t>::max();
  return fault;
}

// Moves `value` towards `wanted` by at most `max_step`.
double approach_value(double value, double wanted, double max_step) {
  return value + std::clamp(wanted - value, -max_step, max_step);
}

}  // namespace

JogSimulation::JogSimulation(const KeyholeScenario& scenario)
    : arm_(at_rest(scenario)),
      injector_(arm_, arm_, armed(scenario.fault)),
      controller_(scenario.arm.model, scenario.gains, scenario.gearboxes,
                  seconds(scenario.arm.control_period)),
      monitor_(scenario.safety),
      trajectory_(scenario.trajectory),
      line_(detail::feed_line(scenario.trajectory, approach_of(scenario).standoff_m)),
      plan_model_(scenario.arm.model.arm()),
      limits_(scenario.safety),
      standoff_m_(approach_of(scenario).standoff_m),
      feed_speed_m_s_(scenario.feed_speed_m_s),
      feed_acceleration_m_s2_(scenario.feed_acceleration_m_s2),
      period_s_(seconds(scenario.arm.control_period)),
      line_start_q_(detail::line_start_q(plan_model_, line_, scenario.arm.initial_q)),
      tip_target_(kinematics::forward_kinematics(plan_model_, arm_.state().q)),
      target_(arm_.state().q) {}

void JogSimulation::step(const OperatorInput& input) noexcept {
  const bool align_pressed = input.align && !previous_.align;
  const bool fault_pressed = input.fault && !previous_.fault;
  const bool joint_mode_pressed = input.joint_mode && !previous_.joint_mode;
  previous_ = input;

  if (stop_tick_.has_value()) {
    arm_.tick();
    injector_.update();
    return;
  }

  if (fault_pressed && !fault_start_tick_.has_value()) {
    fault_start_tick_ = ticks();
    injector_.start_at(ticks());
  }

  switch (mode_) {
    case JogMode::free:
      if (joint_mode_pressed) {
        if (arm_moving()) {
          refusal_ = AlignRefusal::still_moving;
        } else {
          refusal_ = AlignRefusal::none;
          mode_ = JogMode::joint;
          joint_velocity_rad_s_.setZero();
          break;
        }
      }
      if (align_pressed) {
        align();
      } else {
        jog(input.tip_velocity_m_s);
      }
      break;
    case JogMode::joint:
      if (joint_mode_pressed) {
        if (arm_moving()) {
          refusal_ = AlignRefusal::still_moving;
        } else {
          // Free mode takes over where the joints left the tip, keeping the tool's direction.
          refusal_ = AlignRefusal::none;
          mode_ = JogMode::free;
          tip_target_ = kinematics::forward_kinematics(plan_model_, target_);
          tip_velocity_m_s_.setZero();
          break;
        }
      }
      turn_joint(input.joint, input.joint_rate);
      break;
    case JogMode::aligning: {
      // `align` sets the move before it enters this mode; without one there is nothing to do.
      if (!move_.has_value()) {
        mode_ = JogMode::free;
        break;
      }
      const double elapsed_s = arm_.clock().now_seconds() - move_start_s_;
      target_ = move_->at(elapsed_s);
      if (elapsed_s >= move_->duration_s()) {
        mode_ = JogMode::on_line;
        line_depth_m_ = 0.0;
        line_speed_m_s_ = 0.0;
      }
      break;
    }
    case JogMode::on_line:
      if (align_pressed) {
        // Let go of the line only where the free mode can take over safely: at the standoff,
        // outside the guard, and stopped.
        if (line_depth_m_ == 0.0 && line_speed_m_s_ == 0.0) {
          refusal_ = AlignRefusal::none;
          mode_ = JogMode::free;
          tip_target_ = kinematics::forward_kinematics(plan_model_, target_);
          tip_velocity_m_s_.setZero();
          break;
        }
        refusal_ = AlignRefusal::not_at_standoff;
      }
      follow_line(input.feed);
      break;
  }

  const hardware::SensorSample control_reading = injector_.read_sensors();
  const safety::SafetyInput safety_input{injector_.read_safety_sensors(), control_reading, target_,
                                         arm_.clock().ticks()};
  if (monitor_.check(safety_input) != safety::StopCause::none) {
    injector_.engage_brakes();
    stop_tick_ = arm_.clock().ticks();
  } else {
    injector_.command_torques(controller_.update(target_, control_reading.joint_positions_rad));
  }

  arm_.tick();
  injector_.update();
}

void JogSimulation::jog(const Eigen::Vector3d& velocity_m_s) noexcept {
  Eigen::Vector3d wanted = velocity_m_s.allFinite() ? velocity_m_s : Eigen::Vector3d::Zero();
  const double speed = wanted.norm();
  if (speed > kMaxTipSpeedMS) {
    wanted *= kMaxTipSpeedMS / speed;
  }
  const Eigen::Vector3d change = wanted - tip_velocity_m_s_;
  const double max_change = kMaxTipAccelerationMS2 * period_s_;
  tip_velocity_m_s_ +=
      change.norm() > max_change ? (change * (max_change / change.norm())) : change;

  math::Pose candidate = tip_target_;
  candidate.translation() += tip_velocity_m_s_ * period_s_;
  kinematics::IkResult solved = kinematics::solve_ik(plan_model_, candidate, target_);
  if (solved.converged()) {
    // Where a small tip motion needs a large joint motion, slow the tip rather than the joints.
    const double load =
        ((solved.q - target_).array().abs() / (limits_.max_velocity_rad_s.array() * period_s_))
            .maxCoeff();
    if (load > kJointSpeedFraction) {
      tip_velocity_m_s_ *= kJointSpeedFraction / load;
      candidate.translation() = tip_target_.translation() + (tip_velocity_m_s_ * period_s_);
      solved = kinematics::solve_ik(plan_model_, candidate, target_);
    }
  }
  if (solved.converged()) {
    tip_target_ = candidate;
    target_ = solved.q;
  } else {
    // The edge of the reachable space: hold there.
    tip_velocity_m_s_.setZero();
  }
}

bool JogSimulation::arm_moving() const noexcept {
  return !tip_velocity_m_s_.isZero(0.0) || !joint_velocity_rad_s_.isZero(0.0);
}

void JogSimulation::turn_joint(int joint, double rate) noexcept {
  const kinematics::ArmModel::Joints& joints = limits_.arm.joints();
  const double max_step = kMaxJointAccelerationRadS2 * period_s_;
  const double fraction = std::isfinite(rate) ? std::clamp(rate, -1.0, 1.0) : 0.0;
  for (int i = 0; i < kinematics::kJointCount; ++i) {
    const auto index = static_cast<std::size_t>(i);
    const double top_speed = kJointSpeedFraction * limits_.max_velocity_rad_s(i);
    double wanted = i == joint ? fraction * top_speed : 0.0;
    // Slow down in time to stop at the soft limit: v^2 = 2 a d.
    const double margin = limits_.joint_limit_margin_rad + kSoftLimitMarginRad;
    const double room = wanted > 0.0 ? joints[index].upper_rad - margin - target_(i)
                                     : target_(i) - (joints[index].lower_rad + margin);
    const double stoppable = std::sqrt(2.0 * kMaxJointAccelerationRadS2 * std::max(room, 0.0));
    wanted = std::clamp(wanted, -stoppable, stoppable);
    joint_velocity_rad_s_(i) = approach_value(joint_velocity_rad_s_(i), wanted, max_step);
    target_(i) = std::clamp(target_(i) + (joint_velocity_rad_s_(i) * period_s_),
                            joints[index].lower_rad + margin, joints[index].upper_rad - margin);
  }
}

void JogSimulation::align() noexcept {
  if (arm_moving()) {
    refusal_ = AlignRefusal::still_moving;
    return;
  }
  const double duration_s = std::max(
      detail::move_duration_s(target_, line_start_q_, limits_, kAlignSpeedFraction), kMinAlignS);
  move_.emplace(target_, line_start_q_, duration_s);
  if (detail::move_enters_guard(plan_model_, *move_, trajectory_, limits_.corridor.entry_guard_m)) {
    move_.reset();
    refusal_ = AlignRefusal::enters_guard;
    return;
  }
  refusal_ = AlignRefusal::none;
  mode_ = JogMode::aligning;
  move_start_s_ = arm_.clock().now_seconds();
  tip_velocity_m_s_.setZero();
}

void JogSimulation::follow_line(double feed) noexcept {
  const double fraction = std::isfinite(feed) ? std::clamp(feed, -1.0, 1.0) : 0.0;
  line_speed_m_s_ = approach_value(line_speed_m_s_, fraction * feed_speed_m_s_,
                                   feed_acceleration_m_s2_ * period_s_);
  line_depth_m_ += line_speed_m_s_ * period_s_;
  if (line_depth_m_ <= 0.0 || line_depth_m_ >= line_.length_m()) {
    line_depth_m_ = std::clamp(line_depth_m_, 0.0, line_.length_m());
    line_speed_m_s_ = 0.0;
  }
  const kinematics::IkResult solved =
      kinematics::solve_ik(plan_model_, line_.pose_at(line_depth_m_), target_);
  // As in a planned run: holding the last target is the conservative response to a miss, and
  // the monitor sees the tracking error.
  if (solved.converged()) {
    target_ = solved.q;
  }
}

math::Pose JogSimulation::true_tool_pose() const noexcept {
  return kinematics::forward_kinematics(plan_model_, arm_.state().q);
}

void JogSimulation::add_to_hash(base::StateHash& hash) const noexcept {
  arm_.add_to_hash(hash);
  hash.add(static_cast<std::uint64_t>(monitor_.cause()));
  hash.add(static_cast<std::uint64_t>(stop_tick_.value_or(-1)));
  hash.add(static_cast<std::uint64_t>(mode_));
}

const char* to_string(JogMode mode) noexcept {
  switch (mode) {
    case JogMode::free:
      return "free";
    case JogMode::aligning:
      return "aligning";
    case JogMode::on_line:
      return "on_line";
    case JogMode::joint:
      return "joint";
  }
  return "unknown";
}

const char* to_string(AlignRefusal refusal) noexcept {
  switch (refusal) {
    case AlignRefusal::none:
      return "none";
    case AlignRefusal::still_moving:
      return "still_moving";
    case AlignRefusal::enters_guard:
      return "enters_guard";
    case AlignRefusal::not_at_standoff:
      return "not_at_standoff";
  }
  return "unknown";
}

}  // namespace stapes::sim
