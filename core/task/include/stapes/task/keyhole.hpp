#pragma once

#include <Eigen/Geometry>

#include "stapes/kinematics/arm_model.hpp"
#include "stapes/math/rotation.hpp"

namespace stapes::task {

/// The direction a tool points in: the +x axis of the reference arm's tool frame.
[[nodiscard]] Eigen::Vector3d tool_axis(const math::Pose& tool) noexcept;

/// Distance from `point` to the infinite line through the tool tip along its axis.
///
/// This one number is the constraint for both scenarios. Keyhole drilling and a remote centre
/// of motion both require the tool axis to pass through a fixed point; they differ only in
/// how the tip moves.
[[nodiscard]] double axis_distance(const math::Pose& tool, const Eigen::Vector3d& point) noexcept;

/// The pose with its tip at `tip` and its axis along `axis`, turned from `reference` by the
/// smallest rotation that does it, so the roll about the tool axis does not wander.
[[nodiscard]] math::Pose pose_on_axis(const Eigen::Vector3d& tip, const Eigen::Vector3d& axis,
                                      const math::Pose& reference) noexcept;

/// The pose with its tip at `tip` and its axis through `fulcrum`, pointing away from it. This
/// is a remote centre of motion, as in laparoscopy.
///
/// @pre `tip` and `fulcrum` differ.
[[nodiscard]] math::Pose pose_through_fulcrum(const Eigen::Vector3d& tip,
                                              const Eigen::Vector3d& fulcrum,
                                              const math::Pose& reference) noexcept;

/// A planned straight tunnel from `entry` to `target`.
///
/// Patient geometry is not modelled here: the entry and target are given, as a surgical plan
/// would give them.
class KeyholeTrajectory {
 public:
  /// @param reference sets the roll about the tool axis.
  /// @throws std::invalid_argument if the points are not finite, or are too close to define a
  ///         direction.
  KeyholeTrajectory(const Eigen::Vector3d& entry, const Eigen::Vector3d& target,
                    const math::Pose& reference);

  [[nodiscard]] const Eigen::Vector3d& entry() const noexcept { return entry_; }
  [[nodiscard]] const Eigen::Vector3d& target() const noexcept { return target_; }
  [[nodiscard]] const Eigen::Vector3d& direction() const noexcept { return direction_; }
  [[nodiscard]] double length_m() const noexcept { return length_m_; }

  /// The tool pose at `depth_m` along the tunnel, clamped to the tunnel's ends.
  [[nodiscard]] math::Pose pose_at(double depth_m) const noexcept;

  /// How far along the tunnel `point` lies. Negative before the entry, above `length_m()` past
  /// the target.
  [[nodiscard]] double depth_of(const Eigen::Vector3d& point) const noexcept;

  /// Distance from `point` to the tunnel's centre line.
  [[nodiscard]] double radial_distance(const Eigen::Vector3d& point) const noexcept;

 private:
  Eigen::Vector3d entry_;
  Eigen::Vector3d target_;
  Eigen::Vector3d direction_;
  double length_m_;
  math::Pose reference_;
};

/// Depth over time for a feed: accelerate, hold the feed speed, decelerate.
class FeedProfile {
 public:
  /// A move too short to reach `speed_m_s` becomes a triangle with a lower peak.
  /// @throws std::invalid_argument unless all three values are positive and finite.
  FeedProfile(double length_m, double speed_m_s, double acceleration_m_s2);

  [[nodiscard]] double depth_at(double time_s) const noexcept;
  [[nodiscard]] double duration_s() const noexcept { return duration_s_; }

 private:
  double length_m_;
  double peak_speed_m_s_;
  double acceleration_m_s2_;
  double ramp_s_{0.0};
  double duration_s_{0.0};
};

/// Checks a plan before any motion: walks the tunnel in `step_m` increments, solving inverse
/// kinematics from each result, as the follower will at run time. Fills `start` with the
/// configuration at depth zero.
///
/// @return whether every point converged within the joint limits.
[[nodiscard]] bool plan_is_reachable(const kinematics::ArmModel& model,
                                     const KeyholeTrajectory& trajectory,
                                     const kinematics::JointVector& seed, double step_m,
                                     kinematics::JointVector& start) noexcept;

}  // namespace stapes::task
