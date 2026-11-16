#include "planning.hpp"

#include <stdexcept>

#include "stapes/kinematics/forward_kinematics.hpp"

namespace stapes::sim::detail {

namespace {

// How many points of a move are checked against the entry guard.
constexpr int kMoveChecks = 200;

}  // namespace

task::KeyholeTrajectory feed_line(const task::KeyholeTrajectory& tunnel, double standoff_m) {
  return {tunnel.entry() - (standoff_m * tunnel.direction()), tunnel.target(), tunnel.pose_at(0.0)};
}

kinematics::JointVector line_start_q(const kinematics::ArmModel& model,
                                     const task::KeyholeTrajectory& line,
                                     const kinematics::JointVector& seed) {
  kinematics::JointVector start;
  if (!task::plan_is_reachable(model, line, seed, kPlanStepM, start)) {
    throw std::invalid_argument("the planned tunnel is not reachable");
  }
  return start;
}

double move_duration_s(const kinematics::JointVector& from, const kinematics::JointVector& to,
                       const safety::SafetyConfig& limits, double fraction) {
  const double slowest_s = (task::JointMove(from, to, 1.0).peak_speeds_rad_s().array() /
                            limits.max_velocity_rad_s.array())
                               .maxCoeff();
  return slowest_s / fraction;
}

bool move_enters_guard(const kinematics::ArmModel& model, const task::JointMove& move,
                       const task::KeyholeTrajectory& tunnel, double guard_m) {
  for (int i = 0; i <= kMoveChecks; ++i) {
    const double time_s = move.duration_s() * i / kMoveChecks;
    const Eigen::Vector3d tip =
        kinematics::forward_kinematics(model, move.at(time_s)).translation();
    if (tunnel.depth_of(tip) > -guard_m) {
      return true;
    }
  }
  return false;
}

}  // namespace stapes::sim::detail
