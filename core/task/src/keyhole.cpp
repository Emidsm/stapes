#include "stapes/task/keyhole.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "stapes/kinematics/inverse_kinematics.hpp"

namespace stapes::task {

namespace {

constexpr double kMinTunnelLengthM = 1e-6;

}  // namespace

Eigen::Vector3d tool_axis(const math::Pose& tool) noexcept { return tool.linear().col(0); }

double axis_distance(const math::Pose& tool, const Eigen::Vector3d& point) noexcept {
  return tool_axis(tool).cross(point - tool.translation()).norm();
}

math::Pose pose_on_axis(const Eigen::Vector3d& tip, const Eigen::Vector3d& axis,
                        const math::Pose& reference) noexcept {
  math::Pose pose = math::Pose::Identity();
  pose.translation() = tip;
  const Eigen::Quaterniond turn = Eigen::Quaterniond::FromTwoVectors(tool_axis(reference), axis);
  pose.linear() = turn.toRotationMatrix() * reference.linear();
  return pose;
}

math::Pose pose_through_fulcrum(const Eigen::Vector3d& tip, const Eigen::Vector3d& fulcrum,
                                const math::Pose& reference) noexcept {
  return pose_on_axis(tip, (tip - fulcrum).normalized(), reference);
}

KeyholeTrajectory::KeyholeTrajectory(const Eigen::Vector3d& entry, const Eigen::Vector3d& target,
                                     const math::Pose& reference)
    : entry_(entry),
      target_(target),
      direction_((target - entry).normalized()),
      length_m_((target - entry).norm()),
      reference_(reference) {
  if (!entry.allFinite() || !target.allFinite() || !(length_m_ >= kMinTunnelLengthM)) {
    throw std::invalid_argument("a tunnel needs finite, distinct end points");
  }
}

math::Pose KeyholeTrajectory::pose_at(double depth_m) const noexcept {
  const double clamped = std::clamp(depth_m, 0.0, length_m_);
  return pose_on_axis(entry_ + (clamped * direction_), direction_, reference_);
}

double KeyholeTrajectory::depth_of(const Eigen::Vector3d& point) const noexcept {
  return direction_.dot(point - entry_);
}

double KeyholeTrajectory::radial_distance(const Eigen::Vector3d& point) const noexcept {
  return direction_.cross(point - entry_).norm();
}

FeedProfile::FeedProfile(double length_m, double speed_m_s, double acceleration_m_s2)
    : length_m_(length_m), peak_speed_m_s_(speed_m_s), acceleration_m_s2_(acceleration_m_s2) {
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  if (!positive(length_m) || !positive(speed_m_s) || !positive(acceleration_m_s2)) {
    throw std::invalid_argument("feed length, speed and acceleration must be positive");
  }
  peak_speed_m_s_ = std::min(speed_m_s, std::sqrt(length_m * acceleration_m_s2));
  ramp_s_ = peak_speed_m_s_ / acceleration_m_s2;
  duration_s_ = (length_m / peak_speed_m_s_) + ramp_s_;
}

double FeedProfile::depth_at(double time_s) const noexcept {
  if (time_s <= 0.0) {
    return 0.0;
  }
  if (time_s >= duration_s_) {
    return length_m_;
  }
  if (time_s < ramp_s_) {
    return 0.5 * acceleration_m_s2_ * time_s * time_s;
  }
  const double remaining = duration_s_ - time_s;
  if (remaining < ramp_s_) {
    return length_m_ - (0.5 * acceleration_m_s2_ * remaining * remaining);
  }
  return peak_speed_m_s_ * (time_s - (0.5 * ramp_s_));
}

bool plan_is_reachable(const kinematics::ArmModel& model, const KeyholeTrajectory& trajectory,
                       const kinematics::JointVector& seed, double step_m,
                       kinematics::JointVector& start) noexcept {
  if (!(step_m > 0.0)) {
    return false;
  }
  const kinematics::IkResult first = kinematics::solve_ik(model, trajectory.pose_at(0.0), seed);
  if (!first.converged()) {
    return false;
  }
  start = first.q;

  kinematics::JointVector current = first.q;
  double depth = 0.0;
  while (depth < trajectory.length_m()) {
    depth = std::min(depth + step_m, trajectory.length_m());
    const kinematics::IkResult next =
        kinematics::solve_ik(model, trajectory.pose_at(depth), current);
    if (!next.converged()) {
      return false;
    }
    current = next.q;
  }
  return true;
}

}  // namespace stapes::task
