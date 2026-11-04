#pragma once

#include "stapes/base/random.hpp"

namespace stapes::plant {

struct EncoderParams {
  double resolution_rad;
  double noise_stddev_rad;
};

/// An incremental encoder: noise on the true angle, then quantisation to whole counts.
class Encoder {
 public:
  /// @throws std::invalid_argument if the resolution is not positive or the noise negative.
  Encoder(const EncoderParams& params, const base::Rng& rng);

  [[nodiscard]] double sample(double true_angle_rad) noexcept;

 private:
  EncoderParams params_;
  base::Rng rng_;
};

}  // namespace stapes::plant
