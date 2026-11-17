#pragma once

// Planning steps shared by the simulations in this module. Internal: not part of its interface.

#include "stapes/kinematics/arm_model.hpp"
#include "stapes/safety/safety_monitor.hpp"
#include "stapes/task/joint_move.hpp"
#include "stapes/task/keyhole.hpp"

namespace stapes::sim::detail {

/// How finely the straight line is walked when it is checked before a run.
inline constexpr double kPlanStepM = 0.5e-3;

/// The straight line a feed follows from a standoff: the tunnel, extended back by `standoff_m`.
[[nodiscard]] task::KeyholeTrajectory feed_line(const task::KeyholeTrajectory& tunnel,
                                                double standoff_m);

/// The configuration at the start of `line`, found by walking the whole line from `seed` as
/// the follower will at run time.
///
/// @throws std::invalid_argument if any point of the line is not reachable.
[[nodiscard]] kinematics::JointVector line_start_q(const kinematics::ArmModel& model,
                                                   const task::KeyholeTrajectory& line,
                                                   const kinematics::JointVector& seed);

/// The duration of a joint-space move from `from` to `to` that brings the fastest joint,
/// relative to its own limit in `limits`, to `fraction` of that limit.
[[nodiscard]] double move_duration_s(const kinematics::JointVector& from,
                                     const kinematics::JointVector& to,
                                     const safety::SafetyConfig& limits, double fraction);

/// Whether `move` brings the tip inside the monitor's entry guard at any point. The monitor
/// holds the tip to the line inside the guard, so a move that cuts in there before reaching the
/// line would be stopped. The check samples the move; it is not exact between samples.
[[nodiscard]] bool move_enters_guard(const kinematics::ArmModel& model, const task::JointMove& move,
                                     const task::KeyholeTrajectory& tunnel, double guard_m);

}  // namespace stapes::sim::detail
