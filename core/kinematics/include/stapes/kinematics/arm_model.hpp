#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>

#include "stapes/math/rotation.hpp"

namespace stapes::kinematics {

inline constexpr int kJointCount = 6;

/// Fixed-size, so kinematics never touches the heap.
using JointVector = Eigen::Matrix<double, kJointCount, 1>;

/// One revolute joint, described the way URDF does: where the joint sits relative
/// to its parent, and the axis it turns about.
struct RevoluteJoint {
  math::Pose origin;     ///< Parent joint frame to this joint's frame, at zero angle.
  Eigen::Vector3d axis;  ///< Unit axis, in this joint's frame.
  double lower_rad;
  double upper_rad;
};

/// Immutable geometry of a serial 6-joint arm.
class ArmModel {
 public:
  using Joints = std::array<RevoluteJoint, kJointCount>;

  /// @throws std::invalid_argument if an axis is not a unit vector, a limit range
  ///         is empty, or a transform is not rigid.
  ArmModel(const Joints& joints, const math::Pose& tool);

  /// Stapes's reference arm: upper arm 0.30 m, forearm 0.30 m, spherical wrist,
  /// and a 0.10 m tool. Illustrative geometry, not the geometry of any real robot.
  [[nodiscard]] static ArmModel reference();

  [[nodiscard]] const Joints& joints() const noexcept { return joints_; }
  /// Last joint's frame to the tool tip.
  [[nodiscard]] const math::Pose& tool() const noexcept { return tool_; }

  [[nodiscard]] bool within_limits(const JointVector& q) const noexcept;
  [[nodiscard]] JointVector clamp_to_limits(const JointVector& q) const noexcept;

 private:
  Joints joints_;
  math::Pose tool_;
};

/// A configuration of the reference arm away from every singularity.
[[nodiscard]] JointVector reference_home();

}  // namespace stapes::kinematics
