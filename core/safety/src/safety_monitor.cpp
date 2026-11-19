#include "stapes/safety/safety_monitor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "stapes/kinematics/forward_kinematics.hpp"

namespace stapes::safety {

namespace {

// The debounced checks in priority order. Index `i` here is index `i` in the monitor's counts.
constexpr std::array<StopCause, 5> kDebouncedCauses{
    StopCause::tool_force,           StopCause::corridor,       StopCause::tracker_disagreement,
    StopCause::encoder_disagreement, StopCause::tracking_error,
};

// Written so that NaN fails: every comparison with NaN is false.
bool positive(double value) noexcept { return std::isfinite(value) && value > 0.0; }
bool not_negative(double value) noexcept { return std::isfinite(value) && value >= 0.0; }

bool all_positive(const kinematics::JointVector& values) noexcept {
  return values.allFinite() && (values.array() > 0.0).all();
}

double corridor_length(const Corridor& corridor) noexcept {
  return (corridor.target_m - corridor.entry_m).norm();
}

void validate(const SafetyConfig& config) {
  const Corridor& corridor = config.corridor;
  const bool corridor_ok = corridor.entry_m.allFinite() && corridor.target_m.allFinite() &&
                           positive(corridor_length(corridor)) && positive(corridor.radius_m) &&
                           not_negative(corridor.overshoot_margin_m) &&
                           not_negative(corridor.entry_guard_m);
  const bool limits_ok = not_negative(config.joint_limit_margin_rad) && positive(config.period_s) &&
                         all_positive(config.max_velocity_rad_s) &&
                         positive(config.max_tool_force_n) &&
                         positive(config.max_tracker_disagreement_m) &&
                         all_positive(config.max_encoder_disagreement_rad) &&
                         all_positive(config.max_tracking_error_rad);
  const bool counts_ok = config.max_sample_age_ticks >= 1 && config.debounce_checks >= 1;
  if (!corridor_ok || !limits_ok || !counts_ok) {
    throw std::invalid_argument("safety limits must be positive and finite");
  }
}

// A sample is usable if it was taken at most `max_age` ticks ago, and not in the future.
bool stale(std::int64_t stamp, std::int64_t now, std::int64_t max_age) noexcept {
  if (stamp > now) {
    return true;
  }
  // `now - stamp` can overflow a signed integer for corrupted stamps. Unsigned subtraction
  // wraps by definition, and the true difference always fits because `stamp <= now`.
  const std::uint64_t age = static_cast<std::uint64_t>(now) - static_cast<std::uint64_t>(stamp);
  return std::cmp_greater(age, max_age);
}

bool finite(const SafetyInput& input) noexcept {
  return input.safety.link_positions_rad.allFinite() &&
         input.safety.tracked_tool_pose.matrix().allFinite() &&
         input.safety.tool_wrench.allFinite() && input.control.joint_positions_rad.allFinite() &&
         input.control.tool_wrench.allFinite() && input.target.allFinite();
}

// Whether any joint of `a` is further than its own limit from the same joint of `b`.
bool differs_by_more_than(const kinematics::JointVector& a, const kinematics::JointVector& b,
                          const kinematics::JointVector& limit) noexcept {
  return ((a - b).array().abs() > limit.array()).any();
}

}  // namespace

SafetyMonitor::SafetyMonitor(const SafetyConfig& config)
    : config_((validate(config), config)),
      corridor_direction_((config.corridor.target_m - config.corridor.entry_m).normalized()),
      corridor_length_m_(corridor_length(config.corridor)),
      max_link_step_rad_(config.max_velocity_rad_s * config.period_s) {}

StopCause SafetyMonitor::check(const SafetyInput& input) noexcept {
  if (stopped()) {
    return cause_;
  }

  cause_ = immediate_cause(input);
  if (cause_ == StopCause::none && has_previous_links_ &&
      too_fast(input.safety.link_positions_rad)) {
    cause_ = StopCause::joint_velocity;
  }
  if (stopped()) {
    return cause_;
  }
  previous_links_rad_ = input.safety.link_positions_rad;
  has_previous_links_ = true;

  // Every count is updated before any decision, so one condition cannot hide another's history.
  const Violations violations = debounced_violations(input);
  for (std::size_t i = 0; i < kDebouncedChecks; ++i) {
    violation_counts_[i] = violations[i] ? violation_counts_[i] + 1 : 0;
  }
  for (std::size_t i = 0; i < kDebouncedChecks; ++i) {
    if (violation_counts_[i] >= config_.debounce_checks) {
      cause_ = kDebouncedCauses[i];
      break;
    }
  }
  return cause_;
}

bool SafetyMonitor::try_reset(const SafetyInput& input) noexcept {
  if (!stopped()) {
    return true;
  }
  if (immediate_cause(input) != StopCause::none) {
    return false;
  }
  for (const bool violated : debounced_violations(input)) {
    if (violated) {
      return false;
    }
  }
  violation_counts_.fill(0);
  has_previous_links_ = false;
  cause_ = StopCause::none;
  return true;
}

StopCause SafetyMonitor::immediate_cause(const SafetyInput& input) const noexcept {
  if (!finite(input)) {
    return StopCause::non_finite_input;
  }
  if (stale(input.safety.stamp_ticks, input.now_ticks, config_.max_sample_age_ticks) ||
      stale(input.control.stamp_ticks, input.now_ticks, config_.max_sample_age_ticks)) {
    return StopCause::stale_data;
  }
  Eigen::Index row = 0;
  for (const kinematics::RevoluteJoint& joint : config_.arm.joints()) {
    const double angle = input.safety.link_positions_rad(row++);
    if (angle < joint.lower_rad + config_.joint_limit_margin_rad ||
        angle > joint.upper_rad - config_.joint_limit_margin_rad) {
      return StopCause::joint_limit;
    }
  }
  return StopCause::none;
}

bool SafetyMonitor::too_fast(const kinematics::JointVector& links_rad) const noexcept {
  return differs_by_more_than(links_rad, previous_links_rad_, max_link_step_rad_);
}

SafetyMonitor::Violations SafetyMonitor::debounced_violations(
    const SafetyInput& input) const noexcept {
  const kinematics::JointVector& links = input.safety.link_positions_rad;
  const math::Pose& tracked = input.safety.tracked_tool_pose;
  // Where the tip is according to the link encoders and the monitor's own copy of the geometry.
  const Eigen::Vector3d tip_from_links =
      kinematics::forward_kinematics(config_.arm, links).translation();
  return Violations{
      input.safety.tool_wrench.head<3>().norm() > config_.max_tool_force_n,
      outside_corridor(tracked),
      (tracked.translation() - tip_from_links).norm() > config_.max_tracker_disagreement_m,
      differs_by_more_than(input.control.joint_positions_rad, links,
                           config_.max_encoder_disagreement_rad),
      differs_by_more_than(links, input.target, config_.max_tracking_error_rad),
  };
}

bool SafetyMonitor::outside_corridor(const math::Pose& tool) const noexcept {
  const Eigen::Vector3d from_entry = tool.translation() - config_.corridor.entry_m;
  const double depth_m = corridor_direction_.dot(from_entry);
  // Only the far end has a depth limit. Backing out is the safe direction.
  if (depth_m > corridor_length_m_ + config_.corridor.overshoot_margin_m) {
    return true;
  }
  // Short of the guard the tip is outside the patient, still approaching. The approach arrives
  // on the line from further out than the guard, so the radius is in force before the entry.
  if (depth_m <= -config_.corridor.entry_guard_m) {
    return false;
  }
  // The tip against the planned line.
  const double radial_m = corridor_direction_.cross(from_entry).norm();
  // The tool's own axis against the entry point: the shaft must pass through the keyhole. The
  // tool points along the +x axis of its frame.
  const double shaft_at_entry_m = tool.linear().col(0).cross(from_entry).norm();
  return radial_m > config_.corridor.radius_m || shaft_at_entry_m > config_.corridor.radius_m;
}

}  // namespace stapes::safety
