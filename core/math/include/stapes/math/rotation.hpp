#pragma once

#include <Eigen/Geometry>

namespace stapes::math {

/// A rigid transform: the pose of a child frame expressed in a parent frame.
using Pose = Eigen::Isometry3d;

/// Rotation vector (axis times angle, in radians) of a rotation matrix: the SO(3)
/// logarithm. The angle returned is in [0, pi]. Stable near zero rotation.
[[nodiscard]] Eigen::Vector3d rotation_vector(const Eigen::Matrix3d& rotation);

/// Rotation vector, in world coordinates, that carries `actual` onto `target`.
[[nodiscard]] Eigen::Vector3d orientation_error(const Eigen::Matrix3d& target,
                                                const Eigen::Matrix3d& actual);

}  // namespace stapes::math
