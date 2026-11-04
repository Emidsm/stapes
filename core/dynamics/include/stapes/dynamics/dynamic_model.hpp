#pragma once

#include <array>

#include <Eigen/Core>

#include "stapes/kinematics/arm_model.hpp"

namespace stapes::dynamics {

/// Mass properties of one link, in that link's joint frame.
struct LinkInertia {
  double mass_kg;
  Eigen::Vector3d com_m;         ///< Centre of mass.
  Eigen::Matrix3d inertia_kgm2;  ///< Rotational inertia about the centre of mass.
};

/// An arm's geometry together with its mass properties and gravity.
class DynamicModel {
 public:
  using Links = std::array<LinkInertia, kinematics::kJointCount>;

  /// @throws std::invalid_argument if a mass is not positive, or an inertia is not
  ///         physically possible for a rigid body.
  DynamicModel(const kinematics::ArmModel& arm, const Links& links,
               const Eigen::Vector3d& gravity_m_s2);

  /// Mass properties for the reference arm, approximating each link as a cylinder.
  /// Illustrative values, not those of any real robot.
  [[nodiscard]] static DynamicModel reference();

  [[nodiscard]] const kinematics::ArmModel& arm() const noexcept { return arm_; }
  [[nodiscard]] const Links& links() const noexcept { return links_; }
  [[nodiscard]] const Eigen::Vector3d& gravity() const noexcept { return gravity_; }

 private:
  kinematics::ArmModel arm_;
  Links links_;
  Eigen::Vector3d gravity_;
};

}  // namespace stapes::dynamics
