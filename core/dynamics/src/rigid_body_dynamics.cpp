#include "stapes/dynamics/rigid_body_dynamics.hpp"

#include <array>
#include <cstddef>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include "stapes/kinematics/forward_kinematics.hpp"

namespace stapes::dynamics {

namespace {

using kinematics::JointVector;
using kinematics::kJointCount;
using Spatial = Eigen::Matrix<double, 6, 6>;
using SpatialVector = Eigen::Matrix<double, 6, 1>;

constexpr std::size_t kLinks = kJointCount;

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

// Rotation from a link's frame to its parent's, at this joint angle.
Eigen::Matrix3d link_rotation(const kinematics::RevoluteJoint& joint, double angle) {
  return joint.origin.linear() * Eigen::AngleAxisd(angle, joint.axis).toRotationMatrix();
}

// Spatial inertia about the link frame origin, with angular terms first.
Spatial spatial_inertia(const LinkInertia& link) {
  const Eigen::Matrix3d c = skew(link.com_m);
  Spatial inertia;
  inertia.topLeftCorner<3, 3>() = link.inertia_kgm2 + (link.mass_kg * c * c.transpose());
  inertia.topRightCorner<3, 3>() = link.mass_kg * c;
  inertia.bottomLeftCorner<3, 3>() = link.mass_kg * c.transpose();
  inertia.bottomRightCorner<3, 3>() = link.mass_kg * Eigen::Matrix3d::Identity();
  return inertia;
}

// Transforms a spatial motion vector from the parent's frame to the link's.
Spatial motion_transform(const Eigen::Matrix3d& link_to_parent, const Eigen::Vector3d& offset) {
  const Eigen::Matrix3d parent_to_link = link_to_parent.transpose();
  Spatial transform = Spatial::Zero();
  transform.topLeftCorner<3, 3>() = parent_to_link;
  transform.bottomLeftCorner<3, 3>() = -parent_to_link * skew(offset);
  transform.bottomRightCorner<3, 3>() = parent_to_link;
  return transform;
}

}  // namespace

JointVector inverse_dynamics(const DynamicModel& model, const JointVector& q, const JointVector& qd,
                             const JointVector& qdd) {
  const kinematics::ArmModel::Joints& joints = model.arm().joints();
  const DynamicModel::Links& links = model.links();

  std::array<Eigen::Matrix3d, kLinks> rotations;
  std::array<Eigen::Vector3d, kLinks> forces;
  std::array<Eigen::Vector3d, kLinks> moments;

  // Outward: velocities and accelerations, each in its own link's frame. Starting the base
  // with an upward acceleration of -g accounts for gravity on every link at once.
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();
  Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = -model.gravity();

  for (std::size_t i = 0; i < kLinks; ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    const Eigen::Vector3d& axis = joints[i].axis;
    const Eigen::Vector3d offset = joints[i].origin.translation();
    rotations[i] = link_rotation(joints[i], q(row));
    const Eigen::Matrix3d to_link = rotations[i].transpose();

    const Eigen::Vector3d carried_omega = to_link * omega;
    accel = to_link * (accel + alpha.cross(offset) + omega.cross(omega.cross(offset)));
    alpha = (to_link * alpha) + (axis * qdd(row)) + carried_omega.cross(axis * qd(row));
    omega = carried_omega + (axis * qd(row));

    const LinkInertia& link = links[i];
    const Eigen::Vector3d com_accel =
        accel + alpha.cross(link.com_m) + omega.cross(omega.cross(link.com_m));
    forces[i] = link.mass_kg * com_accel;
    moments[i] = (link.inertia_kgm2 * alpha) + omega.cross(link.inertia_kgm2 * omega);
  }

  // Inward: each joint carries its own link's load plus everything beyond it.
  JointVector torques;
  Eigen::Vector3d force_from_child = Eigen::Vector3d::Zero();
  Eigen::Vector3d moment_from_child = Eigen::Vector3d::Zero();

  for (std::size_t i = kLinks; i-- > 0;) {
    const Eigen::Vector3d moment = moments[i] + links[i].com_m.cross(forces[i]) + moment_from_child;
    const Eigen::Vector3d force = forces[i] + force_from_child;
    torques(static_cast<Eigen::Index>(i)) = moment.dot(joints[i].axis);

    force_from_child = rotations[i] * force;
    moment_from_child =
        (rotations[i] * moment) + joints[i].origin.translation().cross(force_from_child);
  }
  return torques;
}

JointVector gravity_torques(const DynamicModel& model, const JointVector& q) {
  return inverse_dynamics(model, q, JointVector::Zero(), JointVector::Zero());
}

MassMatrix mass_matrix(const DynamicModel& model, const JointVector& q) {
  const kinematics::ArmModel::Joints& joints = model.arm().joints();
  const DynamicModel::Links& links = model.links();

  std::array<Spatial, kLinks> transforms;
  std::array<Spatial, kLinks> composite;
  std::array<SpatialVector, kLinks> subspaces;
  for (std::size_t i = 0; i < kLinks; ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    transforms[i] =
        motion_transform(link_rotation(joints[i], q(row)), joints[i].origin.translation());
    composite[i] = spatial_inertia(links[i]);
    subspaces[i] << joints[i].axis, Eigen::Vector3d::Zero();
  }

  MassMatrix mass;
  for (std::size_t i = kLinks; i-- > 0;) {
    if (i > 0) {
      composite[i - 1] += transforms[i].transpose() * composite[i] * transforms[i];
    }
    const auto joint = static_cast<Eigen::Index>(i);
    SpatialVector force = composite[i] * subspaces[i];
    mass(joint, joint) = subspaces[i].dot(force);
    for (std::size_t j = i; j > 0; --j) {
      force = transforms[j].transpose() * force;
      const auto ancestor = static_cast<Eigen::Index>(j - 1);
      const double coupling = force.dot(subspaces[j - 1]);
      mass(joint, ancestor) = coupling;
      mass(ancestor, joint) = coupling;
    }
  }
  return mass;
}

JointVector forward_dynamics(const DynamicModel& model, const JointVector& q, const JointVector& qd,
                             const JointVector& torques) {
  const JointVector bias = inverse_dynamics(model, q, qd, JointVector::Zero());
  return mass_matrix(model, q).ldlt().solve(torques - bias);
}

double kinetic_energy(const DynamicModel& model, const JointVector& q, const JointVector& qd) {
  return 0.5 * qd.dot(mass_matrix(model, q) * qd);
}

double potential_energy(const DynamicModel& model, const JointVector& q) {
  const kinematics::ChainFrames frames = kinematics::chain_frames(model.arm(), q);
  double energy = 0.0;
  for (std::size_t i = 0; i < kLinks; ++i) {
    const Eigen::Vector3d com_world = frames[i] * model.links()[i].com_m;
    energy -= model.links()[i].mass_kg * model.gravity().dot(com_world);
  }
  return energy;
}

}  // namespace stapes::dynamics
