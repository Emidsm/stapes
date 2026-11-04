#pragma once

#include <Eigen/Core>

#include "stapes/dynamics/dynamic_model.hpp"
#include "stapes/kinematics/arm_model.hpp"

namespace stapes::dynamics {

using MassMatrix = Eigen::Matrix<double, kinematics::kJointCount, kinematics::kJointCount>;

/// Joint torques needed to produce an acceleration: the recursive Newton-Euler algorithm.
[[nodiscard]] kinematics::JointVector inverse_dynamics(const DynamicModel& model,
                                                       const kinematics::JointVector& q,
                                                       const kinematics::JointVector& qd,
                                                       const kinematics::JointVector& qdd);

/// Joint torques that hold the arm still against gravity.
[[nodiscard]] kinematics::JointVector gravity_torques(const DynamicModel& model,
                                                      const kinematics::JointVector& q);

/// The joint-space mass matrix: the composite rigid body algorithm.
[[nodiscard]] MassMatrix mass_matrix(const DynamicModel& model, const kinematics::JointVector& q);

/// Joint accelerations produced by joint torques.
[[nodiscard]] kinematics::JointVector forward_dynamics(const DynamicModel& model,
                                                       const kinematics::JointVector& q,
                                                       const kinematics::JointVector& qd,
                                                       const kinematics::JointVector& torques);

[[nodiscard]] double kinetic_energy(const DynamicModel& model, const kinematics::JointVector& q,
                                    const kinematics::JointVector& qd);

/// Gravitational potential energy, relative to the world origin.
[[nodiscard]] double potential_energy(const DynamicModel& model, const kinematics::JointVector& q);

}  // namespace stapes::dynamics
