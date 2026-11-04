#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include <gtest/gtest.h>
#include <Eigen/Cholesky>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/dynamics/rigid_body_dynamics.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"

namespace {

using stapes::dynamics::DynamicModel;
using stapes::dynamics::forward_dynamics;
using stapes::dynamics::gravity_torques;
using stapes::dynamics::inverse_dynamics;
using stapes::dynamics::kinetic_energy;
using stapes::dynamics::mass_matrix;
using stapes::dynamics::MassMatrix;
using stapes::dynamics::potential_energy;
using stapes::kinematics::JointVector;

const DynamicModel kModel = DynamicModel::reference();
constexpr double kGravity = 9.81;

JointVector random_vector(stapes::base::Rng& rng, double magnitude) {
  JointVector v;
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    v(i) = rng.uniform(-magnitude, magnitude);
  }
  return v;
}

// All the mass sits in one point at the tool tip, so results can be worked out by hand.
DynamicModel point_mass_at_tip(double mass_kg) {
  DynamicModel::Links links = kModel.links();
  for (auto& link : links) {
    link.mass_kg = 1e-12;
    link.com_m.setZero();
    link.inertia_kgm2 = 1e-12 * Eigen::Matrix3d::Identity();
  }
  links[5].mass_kg = mass_kg;
  links[5].com_m = Eigen::Vector3d(0.10, 0.0, 0.0);
  return {kModel.arm(), links, kModel.gravity()};
}

// At zero angle the tip is at (0.45, 0, 0.60), and the shoulder axis at height 0.30.
TEST(InverseDynamics, MatchesGravityTorqueWorkedOutByHand) {
  stapes::test::verifies("SRS-DYN-002");
  constexpr double kMass = 2.0;
  const DynamicModel model = point_mass_at_tip(kMass);
  const JointVector torques = gravity_torques(model, JointVector::Zero());

  // Holding a weight out in front at 0.45 m takes that lever arm times the weight, in the
  // direction that lifts it: rotation about +y moves +x downward, so the sign is negative.
  EXPECT_NEAR(torques(1), -0.45 * kMass * kGravity, 1e-9);  // Shoulder.
  EXPECT_NEAR(torques(2), -0.45 * kMass * kGravity, 1e-9);  // Elbow: same horizontal reach.
  EXPECT_NEAR(torques(4), -0.15 * kMass * kGravity, 1e-9);  // Wrist: 0.15 m to the tip.
  EXPECT_NEAR(torques(0), 0.0, 1e-9);                       // Vertical axis: no gravity load.
  EXPECT_NEAR(torques(3), 0.0, 1e-9);                       // Rolls about the arm's own line.
  EXPECT_NEAR(torques(5), 0.0, 1e-9);
}

TEST(InverseDynamics, MatchesCentripetalLoadWorkedOutByHand) {
  // Spinning the base at w with the tip at radius 0.45 needs a centripetal force of
  // m w^2 r toward the axis. On the shoulder, at height 0.30, that force has a moment
  // arm of 0.30 m, and it pulls the mass inward, in -x.
  stapes::test::verifies("SRS-DYN-002");
  constexpr double kMass = 2.0;
  constexpr double kSpeed = 3.0;
  const DynamicModel model = point_mass_at_tip(kMass);
  JointVector qd = JointVector::Zero();
  qd(0) = kSpeed;
  const JointVector moving = inverse_dynamics(model, JointVector::Zero(), qd, JointVector::Zero());
  const JointVector still = gravity_torques(model, JointVector::Zero());
  EXPECT_NEAR(moving(1) - still(1), -0.30 * kMass * kSpeed * kSpeed * 0.45, 1e-9);
}

TEST(InverseDynamics, MatchesGyroscopicLoadWorkedOutByHand) {
  // Energy cannot check this term: w x Iw is perpendicular to w, so it does no work.
  //
  // Only the last link has inertia, diag(Ix, Iy, Iz), with no mass to speak of. At zero angle
  // all frames line up. Spinning the wrist (y) at b and the tool (x) at a gives that link
  //   w = (a, b, 0)   and   alpha = (0, b, 0) x (a, 0, 0) = (0, 0, -ab).
  // The moment needed is I alpha + w x Iw, whose z component is
  //   -Iz ab + ab (Iy - Ix) = ab (Iy - Ix - Iz),
  // and the base joint, about z, is the one that has to supply it.
  stapes::test::verifies("SRS-DYN-002");
  constexpr double kIx = 0.10;
  constexpr double kIy = 0.20;
  constexpr double kIz = 0.25;
  constexpr double kToolSpeed = 2.0;
  constexpr double kWristSpeed = 3.0;

  DynamicModel::Links links = kModel.links();
  for (auto& link : links) {
    link.mass_kg = 1e-12;
    link.com_m.setZero();
    link.inertia_kgm2 = 1e-12 * Eigen::Matrix3d::Identity();
  }
  links[5].inertia_kgm2 = Eigen::Vector3d(kIx, kIy, kIz).asDiagonal();
  const DynamicModel model(kModel.arm(), links, Eigen::Vector3d::Zero());

  JointVector qd = JointVector::Zero();
  qd(4) = kWristSpeed;
  qd(5) = kToolSpeed;
  const JointVector torques = inverse_dynamics(model, JointVector::Zero(), qd, JointVector::Zero());
  EXPECT_NEAR(torques(0), kToolSpeed * kWristSpeed * (kIy - kIx - kIz), 1e-9);
}

TEST(InverseDynamics, GravityMatchesTheJacobianTransposeOfEachLink) {
  // A different method: the virtual work of each link's weight, through a Jacobian of
  // that link's centre of mass.
  stapes::test::verifies("SRS-DYN-002");
  stapes::test::for_all_cases(501, 2000, [](stapes::base::Rng& rng) {
    const JointVector q = stapes::test::random_configuration(kModel.arm(), rng);
    const stapes::kinematics::ChainFrames frames =
        stapes::kinematics::chain_frames(kModel.arm(), q);

    JointVector expected = JointVector::Zero();
    for (std::size_t link = 0; link < 6; ++link) {
      const Eigen::Vector3d com = frames[link] * kModel.links()[link].com_m;
      const Eigen::Vector3d weight = kModel.links()[link].mass_kg * kModel.gravity();
      for (std::size_t joint = 0; joint <= link; ++joint) {
        const Eigen::Vector3d axis = frames[joint].linear() * kModel.arm().joints()[joint].axis;
        const Eigen::Vector3d lever = com - frames[joint].translation();
        expected(static_cast<Eigen::Index>(joint)) -= axis.cross(lever).dot(weight);
      }
    }
    EXPECT_TRUE(gravity_torques(kModel, q).isApprox(expected, 1e-10));
  });
}

TEST(MassMatrix, MatchesInertiaWorkedOutByHand) {
  // A point mass at radius 0.45 from the base axis has m r^2 about it.
  stapes::test::verifies("SRS-DYN-003");
  constexpr double kMass = 2.0;
  const MassMatrix mass = mass_matrix(point_mass_at_tip(kMass), JointVector::Zero());
  EXPECT_NEAR(mass(0, 0), kMass * 0.45 * 0.45, 1e-9);
  // About the shoulder, at a distance of sqrt(0.45^2 + 0.30^2).
  EXPECT_NEAR(mass(1, 1), kMass * ((0.45 * 0.45) + (0.30 * 0.30)), 1e-9);
}

TEST(MassMatrix, MatchesInverseDynamicsColumnByColumn) {
  // The two algorithms share no code path: composite inertias against a full recursion.
  stapes::test::verifies("SRS-DYN-003");
  stapes::test::for_all_cases(503, 1000, [](stapes::base::Rng& rng) {
    const JointVector q = stapes::test::random_configuration(kModel.arm(), rng);
    const MassMatrix mass = mass_matrix(kModel, q);
    const JointVector gravity = gravity_torques(kModel, q);
    for (Eigen::Index column = 0; column < mass.cols(); ++column) {
      const JointVector unit = JointVector::Unit(column);
      const JointVector torque = inverse_dynamics(kModel, q, JointVector::Zero(), unit) - gravity;
      EXPECT_TRUE(mass.col(column).isApprox(torque, 1e-10)) << "column " << column;
    }
  });
}

TEST(MassMatrix, IsSymmetricAndPositiveDefinite) {
  stapes::test::verifies("SRS-DYN-003");
  stapes::test::for_all_cases(505, 2000, [](stapes::base::Rng& rng) {
    const MassMatrix mass =
        mass_matrix(kModel, stapes::test::random_configuration(kModel.arm(), rng));
    EXPECT_TRUE(mass.isApprox(mass.transpose(), 1e-14));
    EXPECT_EQ(mass.llt().info(), Eigen::Success);
  });
}

TEST(ForwardDynamics, InvertsInverseDynamics) {
  stapes::test::verifies("SRS-DYN-004");
  stapes::test::for_all_cases(507, 2000, [](stapes::base::Rng& rng) {
    const JointVector q = stapes::test::random_configuration(kModel.arm(), rng);
    const JointVector qd = random_vector(rng, 3.0);
    const JointVector qdd = random_vector(rng, 10.0);
    const JointVector torques = inverse_dynamics(kModel, q, qd, qdd);
    EXPECT_TRUE(forward_dynamics(kModel, q, qd, torques).isApprox(qdd, 1e-8));
  });
}

// Largest change in total energy over 2 s of unforced motion, by fourth-order Runge-Kutta.
double energy_drift(const JointVector& start_q, const JointVector& start_qd, double step) {
  JointVector q = start_q;
  JointVector qd = start_qd;
  const JointVector no_torque = JointVector::Zero();
  const double initial = kinetic_energy(kModel, q, qd) + potential_energy(kModel, q);
  const int steps = static_cast<int>(std::round(2.0 / step));

  double worst = 0.0;
  for (int i = 0; i < steps; ++i) {
    const JointVector k1v = forward_dynamics(kModel, q, qd, no_torque);
    const JointVector k1p = qd;
    const JointVector k2p = qd + (0.5 * step * k1v);
    const JointVector k2v = forward_dynamics(kModel, q + (0.5 * step * k1p), k2p, no_torque);
    const JointVector k3p = qd + (0.5 * step * k2v);
    const JointVector k3v = forward_dynamics(kModel, q + (0.5 * step * k2p), k3p, no_torque);
    const JointVector k4p = qd + (step * k3v);
    const JointVector k4v = forward_dynamics(kModel, q + (step * k3p), k4p, no_torque);
    q += (step / 6.0) * (k1p + (2.0 * k2p) + (2.0 * k3p) + k4p);
    qd += (step / 6.0) * (k1v + (2.0 * k2v) + (2.0 * k3v) + k4v);
    const double energy = kinetic_energy(kModel, q, qd) + potential_energy(kModel, q);
    worst = std::max(worst, std::abs(energy - initial));
  }
  return worst;
}

// Conservation of energy has no knowledge of how the dynamics are computed, so it catches
// wrong Coriolis or centrifugal terms that static checks cannot see. Some drift always comes
// from the integrator, so the test is how it converges: Runge-Kutta's error falls about
// 16-fold when the step halves, while an error in the model would stay where it is.
TEST(Energy, DriftVanishesAtTheIntegratorsOrderInFreeMotion) {
  stapes::test::verifies("SRS-DYN-005");
  double worst_fine = 0.0;
  stapes::test::for_all_cases(509, 10, [&worst_fine](stapes::base::Rng& rng) {
    constexpr double kRoundoffFloor = 1e-10;
    const JointVector q = stapes::test::random_configuration(kModel.arm(), rng);
    const JointVector qd = random_vector(rng, 2.0);
    const double coarse = energy_drift(q, qd, 1e-3);
    const double fine = energy_drift(q, qd, 5e-4);
    worst_fine = std::max(worst_fine, fine);
    EXPECT_LE(fine, std::max(coarse / 8.0, kRoundoffFloor)) << "coarse " << coarse;
  });
  RecordProperty("worst_energy_drift_j", std::to_string(worst_fine));
}

TEST(Energy, PotentialEnergyMatchesAPointMassWorkedOutByHand) {
  stapes::test::verifies("SRS-DYN-005");
  constexpr double kMass = 2.0;
  EXPECT_NEAR(potential_energy(point_mass_at_tip(kMass), JointVector::Zero()),
              kMass * kGravity * 0.60, 1e-9);
}

}  // namespace
