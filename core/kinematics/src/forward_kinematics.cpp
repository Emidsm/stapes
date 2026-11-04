#include "stapes/kinematics/forward_kinematics.hpp"

#include <cmath>
#include <cstddef>

#include <Eigen/LU>

namespace stapes::kinematics {

ChainFrames chain_frames(const ArmModel& model, const JointVector& q) {
  ChainFrames frames;
  math::Pose current = math::Pose::Identity();

  const ArmModel::Joints& joints = model.joints();
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const double angle = q(static_cast<Eigen::Index>(i));
    current = current * joints[i].origin * Eigen::AngleAxisd(angle, joints[i].axis);
    frames[i] = current;
  }
  frames.back() = current * model.tool();
  return frames;
}

math::Pose forward_kinematics(const ArmModel& model, const JointVector& q) {
  return chain_frames(model, q).back();
}

Jacobian geometric_jacobian(const ArmModel& model, const JointVector& q) {
  const ChainFrames frames = chain_frames(model, q);
  const Eigen::Vector3d tip = frames.back().translation();

  Jacobian jacobian;
  const ArmModel::Joints& joints = model.joints();
  for (std::size_t i = 0; i < joints.size(); ++i) {
    // A joint frame turns about its own axis and origin, so both are the same before
    // and after the joint's rotation, and the stored frame can be used directly.
    const Eigen::Vector3d axis_world = frames[i].linear() * joints[i].axis;
    const Eigen::Vector3d lever_arm = tip - frames[i].translation();

    const auto column = static_cast<Eigen::Index>(i);
    jacobian.col(column).head<3>() = axis_world.cross(lever_arm);
    jacobian.col(column).tail<3>() = axis_world;
  }
  return jacobian;
}

double manipulability(const ArmModel& model, const JointVector& q) {
  return std::abs(geometric_jacobian(model, q).determinant());
}

}  // namespace stapes::kinematics
