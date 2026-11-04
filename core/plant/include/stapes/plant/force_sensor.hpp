#pragma once

#include "stapes/base/random.hpp"
#include "stapes/math/wrench.hpp"

namespace stapes::plant {

struct ForceSensorParams {
  double force_noise_stddev_n;
  double torque_noise_stddev_nm;
  /// Random-walk bias: its standard deviation grows by this much per square root of a second.
  double force_drift_n_per_sqrt_s;
  double torque_drift_nm_per_sqrt_s;
  double scale_error;  ///< 0.01 reads 1 % high.
};

/// A 6-axis force and torque sensor whose zero wanders over time.
class ForceSensor {
 public:
  /// @throws std::invalid_argument if a noise or drift value is negative.
  ForceSensor(const ForceSensorParams& params, const base::Rng& rng);

  /// Advances the drift by `elapsed_s`, then returns one reading of `true_wrench`.
  [[nodiscard]] math::Wrench sample(const math::Wrench& true_wrench, double elapsed_s) noexcept;

  [[nodiscard]] const math::Wrench& bias() const noexcept { return bias_; }

 private:
  ForceSensorParams params_;
  base::Rng rng_;
  math::Wrench bias_{math::Wrench::Zero()};
};

}  // namespace stapes::plant
