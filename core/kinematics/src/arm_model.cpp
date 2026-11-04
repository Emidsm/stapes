#include "stapes/kinematics/arm_model.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace stapes::kinematics {

namespace {

constexpr double kGeometryTolerance = 1e-9;

bool is_rigid(const math::Pose& pose) {
  return pose.matrix().allFinite() && pose.linear().isUnitary(kGeometryTolerance) &&
         pose.linear().determinant() > 0.0;
}

RevoluteJoint revolute(const Eigen::Vector3d& offset_m, const Eigen::Vector3d& axis,
                       double lower_deg, double upper_deg) {
  constexpr double kRadPerDeg = std::numbers::pi / 180.0;
  math::Pose origin = math::Pose::Identity();
  origin.translation() = offset_m;
  return RevoluteJoint{origin, axis, lower_deg * kRadPerDeg, upper_deg * kRadPerDeg};
}

}  // namespace

ArmModel::ArmModel(const Joints& joints, const math::Pose& tool) : joints_(joints), tool_(tool) {
  for (const RevoluteJoint& joint : joints_) {
    if (!is_rigid(joint.origin)) {
      throw std::invalid_argument("joint origin is not a rigid transform");
    }
    if (!joint.axis.allFinite() || std::abs(joint.axis.norm() - 1.0) > kGeometryTolerance) {
      throw std::invalid_argument("joint axis is not a unit vector");
    }
    if (!std::isfinite(joint.lower_rad) || !std::isfinite(joint.upper_rad) ||
        joint.lower_rad >= joint.upper_rad) {
      throw std::invalid_argument("joint limit range is empty");
    }
  }
  if (!is_rigid(tool_)) {
    throw std::invalid_argument("tool transform is not rigid");
  }
}

ArmModel ArmModel::reference() {
  const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();

  math::Pose tool = math::Pose::Identity();
  tool.translation() = Eigen::Vector3d(0.10, 0.0, 0.0);

  // At zero angle the upper arm points up (+z) and the forearm points forward (+x).
  // Joints 4, 5 and 6 intersect at one point, which makes a spherical wrist.
  return ArmModel(
      Joints{
          revolute({0.0, 0.0, 0.20}, z, -170.0, 170.0),  // base yaw
          revolute({0.0, 0.0, 0.10}, y, -120.0, 120.0),  // shoulder pitch
          revolute({0.0, 0.0, 0.30}, y, -80.0, 140.0),   // elbow pitch; straight at -90
          revolute({0.15, 0.0, 0.0}, x, -170.0, 170.0),  // forearm roll
          revolute({0.15, 0.0, 0.0}, y, -120.0, 120.0),  // wrist pitch; singular at 0
          revolute({0.05, 0.0, 0.0}, x, -175.0, 175.0),  // tool roll
      },
      tool);
}

bool ArmModel::within_limits(const JointVector& q) const noexcept {
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const double angle = q(static_cast<Eigen::Index>(i));
    // Written so that NaN, which fails every comparison, is reported as out of limits.
    // The form the check suggests, angle < lower || angle > upper, would let NaN through.
    // NOLINTNEXTLINE(readability-simplify-boolean-expr)
    if (!(angle >= joints_[i].lower_rad && angle <= joints_[i].upper_rad)) {
      return false;
    }
  }
  return true;
}

JointVector ArmModel::clamp_to_limits(const JointVector& q) const noexcept {
  JointVector clamped = q;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    clamped(row) = std::clamp(q(row), joints_[i].lower_rad, joints_[i].upper_rad);
  }
  return clamped;
}

JointVector reference_home() {
  JointVector home;
  home << 0.0, -0.3, 0.6, 0.0, 0.8, 0.0;
  return home;
}

}  // namespace stapes::kinematics
