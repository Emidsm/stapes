#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/plant/encoder.hpp"

namespace {

using stapes::base::Rng;
using stapes::plant::Encoder;
using stapes::plant::EncoderParams;

TEST(Encoder, RoundsToTheNearestCount) {
  stapes::test::verifies("SRS-SEN-001");
  Encoder encoder(EncoderParams{0.01, 0.0}, Rng(1));
  EXPECT_NEAR(encoder.sample(0.123), 0.12, 1e-12);
  EXPECT_NEAR(encoder.sample(0.127), 0.13, 1e-12);
  EXPECT_NEAR(encoder.sample(-0.123), -0.12, 1e-12);
}

TEST(Encoder, IsNeverOffByMoreThanHalfACountWithoutNoise) {
  stapes::test::verifies("SRS-SEN-001");
  constexpr double kResolution = 0.001;
  stapes::test::for_all_cases(611, 5000, [](Rng& rng) {
    Encoder encoder(EncoderParams{kResolution, 0.0}, Rng(1));
    const double angle = rng.uniform(-4.0, 4.0);
    const double reading = encoder.sample(angle);
    EXPECT_LE(std::abs(reading - angle), (kResolution / 2.0) + 1e-12);
    const double counts = reading / kResolution;
    EXPECT_NEAR(counts, std::round(counts), 1e-9);
  });
}

TEST(Encoder, NoiseHasTheConfiguredStandardDeviation) {
  stapes::test::verifies("SRS-SEN-001");
  constexpr double kNoise = 0.01;
  constexpr int kSamples = 200000;
  Encoder encoder(EncoderParams{1e-6, kNoise}, Rng(2));
  double sum = 0.0;
  double sum_squares = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double reading = encoder.sample(0.5) - 0.5;
    sum += reading;
    sum_squares += reading * reading;
  }
  EXPECT_NEAR(sum / kSamples, 0.0, 5.0 * kNoise / std::sqrt(kSamples));
  EXPECT_NEAR(std::sqrt(sum_squares / kSamples), kNoise, 0.02 * kNoise);
}

TEST(Encoder, RejectsInvalidParameters) {
  stapes::test::verifies("SRS-SEN-001");
  EXPECT_THROW(Encoder(EncoderParams{0.0, 0.0}, Rng(1)), std::invalid_argument);
  EXPECT_THROW(Encoder(EncoderParams{0.01, -1.0}, Rng(1)), std::invalid_argument);
}

}  // namespace
