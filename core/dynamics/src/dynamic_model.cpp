#include "stapes/dynamics/dynamic_model.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <Eigen/Eigenvalues>

namespace stapes::dynamics {

namespace {

constexpr double kSymmetryTolerance = 1e-12;

// A real body's principal moments are positive and satisfy the triangle inequality,
// since each is a sum of mass times squared distances over two of the three axes.
bool is_physical_inertia(const Eigen::Matrix3d& inertia) {
  if (!inertia.allFinite() || !inertia.isApprox(inertia.transpose(), kSymmetryTolerance)) {
    return false;
  }
  const Eigen::Vector3d moments =
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(inertia, Eigen::EigenvaluesOnly)
          .eigenvalues();  // Sorted in increasing order.
  return moments(0) > 0.0 && moments(0) + moments(1) >= moments(2);
}

enum class Axis : std::uint8_t { x, y, z };

LinkInertia cylinder(double mass_kg, double length_m, double radius_m, Axis axis,
                     const Eigen::Vector3d& com_m) {
  const double along = 0.5 * mass_kg * radius_m * radius_m;
  const double across = mass_kg * ((3.0 * radius_m * radius_m) + (length_m * length_m)) / 12.0;
  Eigen::Vector3d moments(across, across, across);
  moments(static_cast<Eigen::Index>(axis)) = along;
  return LinkInertia{mass_kg, com_m, moments.asDiagonal()};
}

}  // namespace

DynamicModel::DynamicModel(const kinematics::ArmModel& arm, const Links& links,
                           const Eigen::Vector3d& gravity_m_s2)
    : arm_(arm), links_(links), gravity_(gravity_m_s2) {
  for (const LinkInertia& link : links_) {
    if (!std::isfinite(link.mass_kg) || link.mass_kg <= 0.0) {
      throw std::invalid_argument("link mass must be positive");
    }
    if (!link.com_m.allFinite()) {
      throw std::invalid_argument("link centre of mass must be finite");
    }
    if (!is_physical_inertia(link.inertia_kgm2)) {
      throw std::invalid_argument("link inertia is not physically possible");
    }
  }
  if (!gravity_.allFinite()) {
    throw std::invalid_argument("gravity must be finite");
  }
}

DynamicModel DynamicModel::reference() {
  return DynamicModel(kinematics::ArmModel::reference(),
                      Links{
                          cylinder(4.0, 0.10, 0.06, Axis::z, {0.0, 0.0, 0.05}),
                          cylinder(3.0, 0.30, 0.05, Axis::z, {0.0, 0.0, 0.15}),
                          cylinder(2.0, 0.15, 0.04, Axis::x, {0.075, 0.0, 0.0}),
                          cylinder(1.0, 0.15, 0.03, Axis::x, {0.075, 0.0, 0.0}),
                          cylinder(0.5, 0.05, 0.03, Axis::x, {0.025, 0.0, 0.0}),
                          cylinder(0.4, 0.10, 0.02, Axis::x, {0.05, 0.0, 0.0}),
                      },
                      Eigen::Vector3d(0.0, 0.0, -9.81));
}

}  // namespace stapes::dynamics
