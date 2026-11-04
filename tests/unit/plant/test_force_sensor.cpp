#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/plant/force_sensor.hpp"

namespace {

using stapes::base::Rng;
using stapes::math::Wrench;
using stapes::plant::ForceSensor;
using stapes::plant::ForceSensorParams;

TEST(ForceSensor, IsExactWithNoErrorsConfigured) {
  stapes::test::verifies("SRS-SEN-002");
  ForceSensor sensor(ForceSensorParams{0.0, 0.0, 0.0, 0.0, 0.0}, Rng(1));
  Wrench wrench;
  wrench << 1.0, -2.0, 3.0, 0.1, -0.2, 0.3;
  EXPECT_EQ(sensor.sample(wrench, 0.001), wrench);
}

TEST(ForceSensor, AppliesScaleError) {
  stapes::test::verifies("SRS-SEN-002");
  ForceSensor sensor(ForceSensorParams{0.0, 0.0, 0.0, 0.0, 0.02}, Rng(1));
  const Wrench wrench = Wrench::Constant(10.0);
  EXPECT_TRUE(sensor.sample(wrench, 0.001).isApprox(Wrench::Constant(10.2), 1e-12));
}

TEST(ForceSensor, HasSeparateNoiseForForceAndTorque) {
  stapes::test::verifies("SRS-SEN-002");
  constexpr double kForceNoise = 0.5;
  constexpr double kTorqueNoise = 0.01;
  constexpr int kSamples = 100000;
  ForceSensor sensor(ForceSensorParams{kForceNoise, kTorqueNoise, 0.0, 0.0, 0.0}, Rng(2));
  Wrench sum_squares = Wrench::Zero();
  for (int i = 0; i < kSamples; ++i) {
    const Wrench reading = sensor.sample(Wrench::Zero(), 0.001);
    sum_squares += reading.cwiseProduct(reading);
  }
  const Wrench deviation = (sum_squares / kSamples).cwiseSqrt();
  for (Eigen::Index channel = 0; channel < 3; ++channel) {
    EXPECT_NEAR(deviation(channel), kForceNoise, 0.02 * kForceNoise);
    EXPECT_NEAR(deviation(channel + 3), kTorqueNoise, 0.02 * kTorqueNoise);
  }
}

TEST(ForceSensor, DriftGrowsWithTheSquareRootOfTime) {
  // A random walk's spread after T seconds is rate * sqrt(T), for any step size. Checked
  // across many independent sensors, each running for the same duration.
  stapes::test::verifies("SRS-SEN-002");
  constexpr double kForceDrift = 0.2;
  constexpr double kTorqueDrift = 0.03;
  constexpr int kSensors = 2000;
  constexpr int kSteps = 200;
  constexpr double kStep = 0.05;  // 10 s in total.

  Wrench sum_squares = Wrench::Zero();
  for (int s = 0; s < kSensors; ++s) {
    ForceSensor sensor(ForceSensorParams{0.0, 0.0, kForceDrift, kTorqueDrift, 0.0},
                       Rng(3, static_cast<std::uint64_t>(s)));
    for (int i = 0; i < kSteps; ++i) {
      static_cast<void>(sensor.sample(Wrench::Zero(), kStep));
    }
    sum_squares += sensor.bias().cwiseProduct(sensor.bias());
  }
  const Wrench spread = (sum_squares / kSensors).cwiseSqrt();
  const double expected_force = kForceDrift * std::sqrt(kSteps * kStep);
  const double expected_torque = kTorqueDrift * std::sqrt(kSteps * kStep);
  for (Eigen::Index channel = 0; channel < 3; ++channel) {
    EXPECT_NEAR(spread(channel), expected_force, 0.08 * expected_force);
    EXPECT_NEAR(spread(channel + 3), expected_torque, 0.08 * expected_torque);
  }
}

TEST(ForceSensor, DriftAppearsInTheReading) {
  stapes::test::verifies("SRS-SEN-002");
  ForceSensor sensor(ForceSensorParams{0.0, 0.0, 1.0, 1.0, 0.0}, Rng(4));
  const Wrench reading = sensor.sample(Wrench::Zero(), 1.0);
  EXPECT_FALSE(reading.isZero());
  EXPECT_EQ(reading, sensor.bias());
}

TEST(ForceSensor, RejectsNegativeParameters) {
  stapes::test::verifies("SRS-SEN-002");
  EXPECT_THROW(ForceSensor(ForceSensorParams{-1.0, 0.0, 0.0, 0.0, 0.0}, Rng(1)),
               std::invalid_argument);
  EXPECT_THROW(ForceSensor(ForceSensorParams{0.0, -1.0, 0.0, 0.0, 0.0}, Rng(1)),
               std::invalid_argument);
  EXPECT_THROW(ForceSensor(ForceSensorParams{0.0, 0.0, -1.0, 0.0, 0.0}, Rng(1)),
               std::invalid_argument);
  EXPECT_THROW(ForceSensor(ForceSensorParams{0.0, 0.0, 0.0, -1.0, 0.0}, Rng(1)),
               std::invalid_argument);
}

}  // namespace
