#pragma once

#include <cstdint>

#include "stapes/base/random.hpp"
#include "stapes/kinematics/arm_model.hpp"
#include "stapes/math/rotation.hpp"

namespace stapes::kinematics {

struct IkOptions {
  double position_tolerance_m{1e-6};
  double orientation_tolerance_rad{1e-6};
  int max_iterations{200};
  /// Largest change to any joint in one iteration, which keeps each linearisation of
  /// the kinematics close enough to be useful.
  double max_step_rad{0.25};
  /// Converts orientation error to metres, so that one cost can weigh both kinds of
  /// error. With this value, 1 mrad counts the same as 0.05 mm.
  double orientation_weight_m{0.05};
};

enum class IkStatus : std::uint8_t {
  converged,       ///< Within both tolerances.
  max_iterations,  ///< Ran out of iterations while still improving.
  stalled,         ///< No step reduces the error: unreachable, or a local minimum.
  invalid_input,   ///< The seed or target contains NaN or infinity. Nothing was solved.
};

struct IkResult {
  JointVector q;  ///< Best configuration found. Always within the joint limits.
  IkStatus status;
  int iterations;
  double position_error_m;
  double orientation_error_rad;

  [[nodiscard]] bool converged() const noexcept { return status == IkStatus::converged; }
};

/// Finds joint angles that place the tool tip at `target`, starting from `seed`.
///
/// Uses Levenberg-Marquardt with bound constraints. The solver is local: from a far
/// seed it can stop in a local minimum, and it reports that as a status rather than
/// a false success. It is deterministic and does not allocate.
[[nodiscard]] IkResult solve_ik(const ArmModel& model, const math::Pose& target,
                                const JointVector& seed, const IkOptions& options = {});

/// Runs `solve_ik` from `seed`, then from random configurations until one converges.
///
/// Joint limits create local minima that a single local solve cannot escape. Use this
/// to find a starting configuration far from any known one, and plain `solve_ik` to
/// track a moving target from the previous solution.
///
/// Returns the first converged result. If none converges, returns the result closest
/// to the target. `iterations` counts all starts. Deterministic for a given `rng`
/// state, and does not allocate.
[[nodiscard]] IkResult solve_ik_multistart(const ArmModel& model, const math::Pose& target,
                                           const JointVector& seed, base::Rng& rng,
                                           int max_starts = 32, const IkOptions& options = {});

}  // namespace stapes::kinematics
