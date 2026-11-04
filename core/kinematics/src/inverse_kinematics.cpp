#include "stapes/kinematics/inverse_kinematics.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>

#include <Eigen/Cholesky>

#include "stapes/kinematics/forward_kinematics.hpp"

namespace stapes::kinematics {

namespace {

using Residual = Eigen::Matrix<double, 6, 1>;
using Hessian = Eigen::Matrix<double, kJointCount, kJointCount>;

constexpr double kInitialDamping = 1e-3;
constexpr double kMinDamping = 1e-12;
constexpr double kMaxDamping = 1e6;
constexpr double kDampingDecrease = 0.25;
constexpr double kDampingIncrease = 4.0;
constexpr double kNegligibleImprovement = 1e-10;  // As a fraction of the current cost.

struct Evaluation {
  Residual residual;
  double position_error_m;
  double orientation_error_rad;
  double cost;
};

Evaluation evaluate(const ArmModel& model, const math::Pose& target, const JointVector& q,
                    const IkOptions& options) {
  const math::Pose actual = forward_kinematics(model, q);
  const Eigen::Vector3d position_error = target.translation() - actual.translation();
  const Eigen::Vector3d rotation_error = math::orientation_error(target.linear(), actual.linear());

  Evaluation evaluation{};
  evaluation.residual.head<3>() = position_error;
  evaluation.residual.tail<3>() = options.orientation_weight_m * rotation_error;
  evaluation.position_error_m = position_error.norm();
  evaluation.orientation_error_rad = rotation_error.norm();
  evaluation.cost = evaluation.residual.squaredNorm();
  return evaluation;
}

bool within_tolerance(const Evaluation& evaluation, const IkOptions& options) {
  return evaluation.position_error_m <= options.position_tolerance_m &&
         evaluation.orientation_error_rad <= options.orientation_tolerance_rad;
}

// A joint resting on a limit and pushed further out cannot contribute. Removing its
// column lets the other joints solve for the remaining error; without this the step
// is dominated by motion that the clamp then throws away.
void freeze_joints_at_limits(const ArmModel& model, const JointVector& q, Jacobian& jacobian,
                             JointVector& gradient) {
  const ArmModel::Joints& joints = model.joints();
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto column = static_cast<Eigen::Index>(i);
    const bool pushed_above = q(column) >= joints[i].upper_rad && gradient(column) > 0.0;
    const bool pushed_below = q(column) <= joints[i].lower_rad && gradient(column) < 0.0;
    if (pushed_above || pushed_below) {
      jacobian.col(column).setZero();
      gradient(column) = 0.0;
    }
  }
}

JointVector limit_step(const JointVector& step, double max_step_rad) {
  const double largest = step.cwiseAbs().maxCoeff();
  return largest > max_step_rad ? JointVector(step * (max_step_rad / largest)) : step;
}

IkResult make_result(const JointVector& q, IkStatus status, int iterations,
                     const Evaluation& evaluation) {
  return IkResult{q, status, iterations, evaluation.position_error_m,
                  evaluation.orientation_error_rad};
}

}  // namespace

IkResult solve_ik(const ArmModel& model, const math::Pose& target, const JointVector& seed,
                  const IkOptions& options) {
  if (!seed.allFinite() || !target.matrix().allFinite()) {
    constexpr double kUnknownError = std::numeric_limits<double>::infinity();
    return IkResult{model.clamp_to_limits(JointVector::Zero()), IkStatus::invalid_input, 0,
                    kUnknownError, kUnknownError};
  }

  JointVector q = model.clamp_to_limits(seed);
  Evaluation current = evaluate(model, target, q, options);
  double damping = kInitialDamping;

  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    if (within_tolerance(current, options)) {
      return make_result(q, IkStatus::converged, iteration, current);
    }

    Jacobian jacobian = geometric_jacobian(model, q);
    jacobian.bottomRows<3>() *= options.orientation_weight_m;
    JointVector gradient = jacobian.transpose() * current.residual;
    freeze_joints_at_limits(model, q, jacobian, gradient);
    const Hessian hessian = jacobian.transpose() * jacobian;

    // Levenberg-Marquardt: large damping gives a short, safe gradient step, and small
    // damping gives a fast Gauss-Newton step. Raise it until a step lowers the cost.
    bool improved = false;
    bool negligible = false;
    while (!improved && damping <= kMaxDamping) {
      const Hessian damped = hessian + (damping * Hessian::Identity());
      const JointVector step = limit_step(damped.ldlt().solve(gradient), options.max_step_rad);
      const JointVector candidate = model.clamp_to_limits(q + step);
      const Evaluation next = evaluate(model, target, candidate, options);

      if (next.cost < current.cost) {
        negligible = current.cost - next.cost <= kNegligibleImprovement * current.cost;
        q = candidate;
        current = next;
        damping = std::max(damping * kDampingDecrease, kMinDamping);
        improved = true;
      } else {
        damping *= kDampingIncrease;
      }
    }

    // Failed attempts can keep yielding ever smaller gains until the iteration budget
    // runs out, so a gain too small to matter also counts as stalled.
    if (!improved || (negligible && !within_tolerance(current, options))) {
      return make_result(q, IkStatus::stalled, iteration + 1, current);
    }
  }

  const IkStatus status =
      within_tolerance(current, options) ? IkStatus::converged : IkStatus::max_iterations;
  return make_result(q, status, options.max_iterations, current);
}

IkResult solve_ik_multistart(const ArmModel& model, const math::Pose& target,
                             const JointVector& seed, base::Rng& rng, int max_starts,
                             const IkOptions& options) {
  const auto weighted_error = [&options](const IkResult& result) {
    const double orientation = options.orientation_weight_m * result.orientation_error_rad;
    return (result.position_error_m * result.position_error_m) + (orientation * orientation);
  };

  IkResult best = solve_ik(model, target, seed, options);
  int total_iterations = best.iterations;

  for (int start = 1;
       start < max_starts && !best.converged() && best.status != IkStatus::invalid_input; ++start) {
    JointVector restart;
    Eigen::Index row = 0;
    for (const RevoluteJoint& joint : model.joints()) {
      restart(row++) = rng.uniform(joint.lower_rad, joint.upper_rad);
    }

    const IkResult attempt = solve_ik(model, target, restart, options);
    total_iterations += attempt.iterations;
    if (attempt.converged() || weighted_error(attempt) < weighted_error(best)) {
      best = attempt;
    }
  }

  best.iterations = total_iterations;
  return best;
}

}  // namespace stapes::kinematics
