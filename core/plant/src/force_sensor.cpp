#include "stapes/plant/force_sensor.hpp"

#include <cmath>
#include <stdexcept>

namespace stapes::plant {

ForceSensor::ForceSensor(const ForceSensorParams& params, const base::Rng& rng)
    : params_(params), rng_(rng) {
  if (!(params.force_noise_stddev_n >= 0.0) || !(params.torque_noise_stddev_nm >= 0.0) ||
      !(params.force_drift_n_per_sqrt_s >= 0.0) || !(params.torque_drift_nm_per_sqrt_s >= 0.0)) {
    throw std::invalid_argument("force sensor noise and drift must not be negative");
  }
}

math::Wrench ForceSensor::sample(const math::Wrench& true_wrench, double elapsed_s) noexcept {
  const double root_elapsed = std::sqrt(elapsed_s);
  math::Wrench reading = (1.0 + params_.scale_error) * true_wrench;
  for (Eigen::Index channel = 0; channel < reading.size(); ++channel) {
    const bool is_force = channel < 3;
    const double drift =
        is_force ? params_.force_drift_n_per_sqrt_s : params_.torque_drift_nm_per_sqrt_s;
    const double noise = is_force ? params_.force_noise_stddev_n : params_.torque_noise_stddev_nm;
    bias_(channel) += drift * root_elapsed * rng_.gaussian();
    reading(channel) += bias_(channel) + (noise * rng_.gaussian());
  }
  return reading;
}

}  // namespace stapes::plant
