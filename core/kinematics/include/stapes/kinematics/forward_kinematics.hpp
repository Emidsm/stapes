#pragma once

#include <array>

#include <Eigen/Core>

#include "stapes/kinematics/arm_model.hpp"
#include "stapes/math/rotation.hpp"

namespace stapes::kinematics {

/// Maps joint velocities to the tool tip's twist in world coordinates:
/// rows 0-2 are linear velocity (m/s), rows 3-5 are angular velocity (rad/s).
using Jacobian = Eigen::Matrix<double, 6, kJointCount>;

/// World pose of every joint frame, followed by the tool tip as the last element.
using ChainFrames = std::array<math::Pose, kJointCount + 1>;

[[nodiscard]] ChainFrames chain_frames(const ArmModel& model, const JointVector& q);

/// World pose of the tool tip.
[[nodiscard]] math::Pose forward_kinematics(const ArmModel& model, const JointVector& q);

/// Geometric Jacobian at the tool tip.
[[nodiscard]] Jacobian geometric_jacobian(const ArmModel& model, const JointVector& q);

/// Yoshikawa's measure for a square Jacobian, |det J|. Zero at a singularity, where
/// the arm loses the ability to move the tool in some direction.
[[nodiscard]] double manipulability(const ArmModel& model, const JointVector& q);

}  // namespace stapes::kinematics
