#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/control/pid_controller.hpp"

namespace {

using stapes::control::PidController;
using stapes::control::PidGains;

constexpr double kPeriod = 0.001;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

TEST(PidController, ProportionalTermScalesTheError) {
  stapes::test::verifies("SRS-CTL-001");
  PidController pid(PidGains{10.0, 0.0, 0.0, 100.0, 0.0}, kPeriod);
  EXPECT_DOUBLE_EQ(pid.update(1.5, 1.0), 5.0);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 1.5), -5.0);
}

TEST(PidController, IntegralAccumulatesErrorOverTime) {
  stapes::test::verifies("SRS-CTL-001");
  PidController pid(PidGains{0.0, 100.0, 0.0, 100.0, 0.0}, kPeriod);
  EXPECT_DOUBLE_EQ(pid.update(2.0, 0.0), 100.0 * 2.0 * kPeriod);
  EXPECT_DOUBLE_EQ(pid.update(2.0, 0.0), 2.0 * 100.0 * 2.0 * kPeriod);
  EXPECT_DOUBLE_EQ(pid.integral(), 2.0 * 100.0 * 2.0 * kPeriod);
}

TEST(PidController, DerivativeOpposesTheMeasuredRate) {
  stapes::test::verifies("SRS-CTL-001");
  PidController pid(PidGains{0.0, 0.0, 2.0, 100.0, 0.0}, kPeriod);
  EXPECT_EQ(pid.update(0.0, 0.0), 0.0) << "no history yet, so no rate";
  EXPECT_DOUBLE_EQ(pid.update(0.0, 0.01), -2.0 * 0.01 / kPeriod);
}

TEST(PidController, FeedforwardAddsToTheOutput) {
  stapes::test::verifies("SRS-CTL-001");
  PidController pid(PidGains{10.0, 0.0, 0.0, 100.0, 0.0}, kPeriod);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, 7.0), 17.0);
}

TEST(PidController, ASetpointStepCausesNoDerivativeKick) {
  stapes::test::verifies("SRS-CTL-002");
  PidController pid(PidGains{10.0, 0.0, 5.0, 1000.0, 0.0}, kPeriod);
  static_cast<void>(pid.update(0.0, 0.0));
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0), 10.0) << "only the proportional term may react";
}

TEST(PidController, DerivativeFilterSmoothsAOneCountJump) {
  stapes::test::verifies("SRS-CTL-002");
  constexpr double kFilter = 0.002;
  PidController pid(PidGains{0.0, 0.0, 1.0, 1000.0, kFilter}, kPeriod);
  static_cast<void>(pid.update(0.0, 0.0));
  const double raw_rate = 0.01 / kPeriod;
  const double first = pid.update(0.0, 0.01);
  const double blend = kFilter / (kFilter + kPeriod);
  EXPECT_DOUBLE_EQ(first, -(1.0 - blend) * raw_rate);
  // With the measurement steady again, the filtered rate decays rather than dropping to 0.
  const double second = pid.update(0.0, 0.01);
  EXPECT_DOUBLE_EQ(second, blend * first);
}

TEST(PidController, OutputNeverExceedsTheLimit) {
  stapes::test::verifies("SRS-CTL-003");
  PidController pid(PidGains{1000.0, 0.0, 0.0, 5.0, 0.0}, kPeriod);
  EXPECT_EQ(pid.update(1.0, 0.0), 5.0);
  EXPECT_EQ(pid.update(-1.0, 0.0), -5.0);
  EXPECT_EQ(pid.update(0.0, 0.0, 1000.0), 5.0);
}

TEST(PidController, IntegralDoesNotWindUpWhileSaturated) {
  stapes::test::verifies("SRS-CTL-003");
  PidController pid(PidGains{1000.0, 1000.0, 0.0, 5.0, 0.0}, kPeriod);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_EQ(pid.update(1.0, 0.0), 5.0);
  }
  EXPECT_EQ(pid.integral(), 0.0);
  // Having not wound up, the output follows the error down at once.
  EXPECT_DOUBLE_EQ(pid.update(0.001, 0.0), 1000.0 * 0.001 + (1000.0 * 0.001 * kPeriod));
}

TEST(PidController, NegativeSaturationAlsoStopsTheIntegral) {
  stapes::test::verifies("SRS-CTL-003");
  PidController pid(PidGains{1000.0, 1000.0, 0.0, 5.0, 0.0}, kPeriod);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(pid.update(-1.0, 0.0), -5.0);
  }
  EXPECT_EQ(pid.integral(), 0.0);
}

TEST(PidController, IntegralCanUnwindWhileSaturated) {
  // Saturated high by feedforward, but with a negative error: integrating reduces the output.
  stapes::test::verifies("SRS-CTL-003");
  PidController pid(PidGains{0.0, 1000.0, 0.0, 5.0, 0.0}, kPeriod);
  EXPECT_EQ(pid.update(-1.0, 0.0, 100.0), 5.0);
  EXPECT_LT(pid.integral(), 0.0);
}

TEST(PidController, ANaNInputGivesZeroAndLeavesTheStateAlone) {
  stapes::test::verifies("SRS-CTL-004");
  PidController pid(PidGains{10.0, 100.0, 1.0, 100.0, 0.0}, kPeriod);
  static_cast<void>(pid.update(1.0, 0.0));
  const double integral = pid.integral();

  EXPECT_EQ(pid.update(kNaN, 0.0), 0.0);
  EXPECT_EQ(pid.update(1.0, kNaN), 0.0);
  EXPECT_EQ(pid.update(1.0, 0.0, kNaN), 0.0);
  EXPECT_EQ(pid.integral(), integral);

  // The next good sample computes as if the bad ones never happened.
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0), 10.0 + integral + (100.0 * kPeriod));
}

TEST(PidController, ResetClearsItsHistory) {
  stapes::test::verifies("SRS-CTL-005");
  PidController pid(PidGains{0.0, 100.0, 1.0, 100.0, 0.0}, kPeriod);
  static_cast<void>(pid.update(1.0, 0.0));
  static_cast<void>(pid.update(1.0, 0.5));
  pid.reset();
  EXPECT_EQ(pid.integral(), 0.0);
  // With no history, a large measurement causes no derivative action.
  EXPECT_DOUBLE_EQ(pid.update(5.0, 5.0), 0.0);
}

TEST(PidController, ResetClearsTheFilteredRate) {
  stapes::test::verifies("SRS-CTL-005");
  PidController pid(PidGains{0.0, 0.0, 1.0, 1000.0, 0.01}, kPeriod);
  static_cast<void>(pid.update(0.0, 0.0));
  static_cast<void>(pid.update(0.0, 1.0));
  pid.reset();
  static_cast<void>(pid.update(0.0, 1.0));
  EXPECT_EQ(pid.update(0.0, 1.0), 0.0) << "a stale rate survived the reset";
}

TEST(PidController, RejectsInvalidParameters) {
  stapes::test::verifies("SRS-CTL-005");
  EXPECT_THROW(PidController(PidGains{-1.0, 0.0, 0.0, 1.0, 0.0}, kPeriod), std::invalid_argument);
  EXPECT_THROW(PidController(PidGains{0.0, -1.0, 0.0, 1.0, 0.0}, kPeriod), std::invalid_argument);
  EXPECT_THROW(PidController(PidGains{0.0, 0.0, -1.0, 1.0, 0.0}, kPeriod), std::invalid_argument);
  EXPECT_THROW(PidController(PidGains{0.0, 0.0, 0.0, 0.0, 0.0}, kPeriod), std::invalid_argument);
  EXPECT_THROW(PidController(PidGains{0.0, 0.0, 0.0, 1.0, -1.0}, kPeriod), std::invalid_argument);
  EXPECT_THROW(PidController(PidGains{0.0, 0.0, 0.0, 1.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_THROW(PidController(PidGains{kNaN, 0.0, 0.0, 1.0, 0.0}, kPeriod), std::invalid_argument);
}

}  // namespace
