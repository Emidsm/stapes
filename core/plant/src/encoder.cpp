#include "stapes/plant/encoder.hpp"

#include <cmath>
#include <stdexcept>

namespace stapes::plant {

Encoder::Encoder(const EncoderParams& params, const base::Rng& rng) : params_(params), rng_(rng) {
  if (!(params.resolution_rad > 0.0) || !(params.noise_stddev_rad >= 0.0)) {
    throw std::invalid_argument("encoder needs a positive resolution and non-negative noise");
  }
}

double Encoder::sample(double true_angle_rad) noexcept {
  const double noisy = true_angle_rad + (params_.noise_stddev_rad * rng_.gaussian());
  return std::round(noisy / params_.resolution_rad) * params_.resolution_rad;
}

}  // namespace stapes::plant
