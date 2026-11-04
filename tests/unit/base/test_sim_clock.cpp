#include <chrono>
#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/base/sim_clock.hpp"

namespace {

using stapes::base::SimClock;
using std::chrono::hours;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;

TEST(SimClock, StartsAtZero) {
  stapes::test::verifies("SRS-SIM-001");
  const SimClock clock(milliseconds(1));
  EXPECT_EQ(clock.ticks(), 0);
  EXPECT_EQ(clock.now(), nanoseconds(0));
}

TEST(SimClock, HasNoDriftAfterAnHourAtOneKilohertz) {
  stapes::test::verifies("SRS-SIM-001");
  SimClock clock(milliseconds(1));
  const std::int64_t ticks_per_hour = 3'600'000;
  for (std::int64_t i = 0; i < ticks_per_hour; ++i) {
    clock.advance();
  }
  EXPECT_EQ(clock.now(), hours(1));
}

TEST(SimClock, AccumulatingFloatingPointWouldDrift) {
  // Negative control: shows the failure that integer ticks exist to prevent.
  stapes::test::verifies("SRS-SIM-001");
  double accumulated_seconds = 0.0;
  for (int i = 0; i < 3'600'000; ++i) {
    accumulated_seconds += 0.001;
  }
  EXPECT_NE(accumulated_seconds, 3600.0);
}

TEST(SimClock, ReportsSeconds) {
  stapes::test::verifies("SRS-SIM-001");
  SimClock clock(milliseconds(250));
  clock.advance();
  clock.advance();
  EXPECT_DOUBLE_EQ(clock.now_seconds(), 0.5);
  EXPECT_EQ(clock.step(), milliseconds(250));
}

TEST(SimClock, RejectsNonPositiveStep) {
  stapes::test::verifies("SRS-SIM-002");
  EXPECT_THROW(SimClock{nanoseconds(0)}, std::invalid_argument);
  EXPECT_THROW(SimClock{nanoseconds(-1)}, std::invalid_argument);
}

}  // namespace
