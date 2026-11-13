// Runs every fault through the complete system: robot, controller and monitor. These tests are
// about outcomes, not which check fires, since several may be correct for one fault.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "fault_scenarios.hpp"
#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"

namespace {

using stapes::faults::Fault;
using stapes::faults::FaultKind;
using stapes::safety::StopCause;
using stapes::sim::KeyholeScenario;
using stapes::sim::KeyholeSimulation;
using stapes::test::kRunTicks;
using stapes::test::run_scenario;
using stapes::test::RunResult;
using stapes::test::without_limits;

constexpr double kMicrometresPerMetre = 1e6;
constexpr double kMaxExcursionM = 0.5e-3;
constexpr std::array<std::int64_t, 2> kStartTicks{600, 1400};

struct CampaignTotals {
  int runs{0};
  int stops{0};
  double worst_excursion_m{0.0};
  std::int64_t worst_latency_ticks{0};
};

void run_case(const Fault& fault, std::uint64_t seed, CampaignTotals& totals) {
  KeyholeScenario scenario = KeyholeScenario::reference(seed);
  scenario.fault = fault;
  const RunResult result = run_scenario(scenario, kRunTicks);
  ++totals.runs;
  totals.worst_excursion_m = std::max(totals.worst_excursion_m, result.max_radial_m);
  if (result.stop_tick.has_value()) {
    ++totals.stops;
    totals.worst_latency_ticks =
        std::max(totals.worst_latency_ticks, *result.stop_tick - fault.start_tick);
  }
  // Whether or not a fault was detected, the patient must not be harmed.
  EXPECT_LT(result.max_radial_m, kMaxExcursionM);
  EXPECT_LT(result.max_axis_offset_m, kMaxExcursionM);
  EXPECT_LT(result.final_depth_m, scenario.trajectory.length_m() + kMaxExcursionM);
}

void run_single_joint_campaign(FaultKind kind, const char* name) {
  CampaignTotals totals;
  for (std::size_t joint = 0; joint < stapes::kinematics::kJointCount; ++joint) {
    for (const std::int64_t start : kStartTicks) {
      for (const std::uint64_t seed : {31U, 32U}) {
        SCOPED_TRACE("joint " + std::to_string(joint) + " start " + std::to_string(start) +
                     " seed " + std::to_string(seed));
        run_case(Fault{kind, start, joint, 0}, seed, totals);
      }
    }
  }
  const std::string prefix(name);
  ::testing::Test::RecordProperty(prefix + "_runs", totals.runs);
  ::testing::Test::RecordProperty(prefix + "_stops", totals.stops);
  ::testing::Test::RecordProperty(prefix + "_worst_excursion_um",
                                  std::to_string(totals.worst_excursion_m * kMicrometresPerMetre));
  ::testing::Test::RecordProperty(prefix + "_worst_latency_ms",
                                  std::to_string(totals.worst_latency_ticks));
  // A joint can move too little in the time left for a fault to matter. Most runs must stop.
  EXPECT_GE(totals.stops, totals.runs * 3 / 4);
}

TEST(FaultCampaign, AFrozenMotorEncoderIsContainedOnEveryJoint) {
  stapes::test::verifies("SRS-SAF-019");
  run_single_joint_campaign(FaultKind::frozen_motor_encoder, "frozen_motor_encoder");
}

TEST(FaultCampaign, AFrozenLinkEncoderIsContainedOnEveryJoint) {
  stapes::test::verifies("SRS-SAF-019");
  run_single_joint_campaign(FaultKind::frozen_link_encoder, "frozen_link_encoder");
}

TEST(FaultCampaign, AStuckActuatorIsContainedOnEveryJoint) {
  stapes::test::verifies("SRS-SAF-019");
  run_single_joint_campaign(FaultKind::stuck_actuator, "stuck_actuator");
}

TEST(FaultCampaign, AFrozenTrackerStopsTheRobot) {
  stapes::test::verifies("SRS-SAF-019");
  CampaignTotals totals;
  for (const std::int64_t start : kStartTicks) {
    for (const std::uint64_t seed : {31U, 32U}) {
      run_case(Fault{FaultKind::frozen_tracker, start, 0, 0}, seed, totals);
    }
  }
  EXPECT_EQ(totals.stops, totals.runs);
  RecordProperty("frozen_tracker_worst_latency_ms", std::to_string(totals.worst_latency_ticks));
}

TEST(FaultCampaign, ADelayBeyondTheLimitStopsInTheSameTick) {
  stapes::test::verifies("SRS-SAF-020");
  const KeyholeScenario reference = KeyholeScenario::reference(1);
  const auto limit = static_cast<int>(reference.safety.max_sample_age_ticks);
  for (const int delay : {limit + 1, 2 * limit, 50, 200}) {
    SCOPED_TRACE("delay " + std::to_string(delay));
    KeyholeScenario scenario = reference;
    scenario.fault = Fault{FaultKind::comms_delay, 800, 0, delay};
    const RunResult result = run_scenario(scenario, kRunTicks);
    EXPECT_EQ(result.cause, StopCause::stale_data);
    ASSERT_TRUE(result.stop_tick.has_value());
    EXPECT_EQ(*result.stop_tick, 800);
  }
}

TEST(FaultCampaign, ADelayWithinTheLimitIsToleratedAndStaysOnTheLine) {
  stapes::test::verifies("SRS-SAF-020");
  const KeyholeScenario reference = KeyholeScenario::reference(1);
  const auto limit = static_cast<int>(reference.safety.max_sample_age_ticks);
  for (int delay = 1; delay <= limit; ++delay) {
    SCOPED_TRACE("delay " + std::to_string(delay));
    KeyholeScenario scenario = reference;
    scenario.fault = Fault{FaultKind::comms_delay, 800, 0, delay};
    const RunResult result = run_scenario(scenario, kRunTicks);
    EXPECT_EQ(result.cause, StopCause::none);
    EXPECT_LT(result.max_radial_m, 0.2e-3);
  }
}

TEST(FaultCampaign, ExcessForceStopsTheRobot) {
  stapes::test::verifies("SRS-SAF-021");
  KeyholeScenario scenario = KeyholeScenario::reference(41);
  const double force = 2.0 * scenario.safety.max_tool_force_n;
  const RunResult result = stapes::test::run_with_thrust(scenario, force);
  EXPECT_NE(result.cause, StopCause::none);
  RecordProperty("excess_force_stop_cause", stapes::safety::to_string(result.cause));
  EXPECT_LT(result.max_radial_m, kMaxExcursionM);
}

TEST(FaultCampaign, AThrustTheLoopCannotHoldIsStoppedNotIgnored) {
  // Joint-level control cannot see gearbox flex, so a strong thrust pushes the tip off the line.
  // The tracker sees it, and the monitor stops before the patient comes to harm.
  stapes::test::verifies("SRS-SAF-021");
  const RunResult result = stapes::test::run_with_thrust(KeyholeScenario::reference(42), 5.0);
  EXPECT_NE(result.cause, StopCause::none);
  EXPECT_LT(result.max_radial_m, kMaxExcursionM);
}

TEST(AfterAStop, TheMotorsAreOffAndTheToolStaysPut) {
  stapes::test::verifies("SRS-SAF-022");
  KeyholeScenario scenario = KeyholeScenario::reference(43);
  scenario.fault = Fault{FaultKind::frozen_motor_encoder, 800, 1, 0};
  KeyholeSimulation sim(scenario);
  for (int i = 0; i < 2000 && !sim.stop_tick().has_value(); ++i) {
    sim.step();
  }
  ASSERT_TRUE(sim.stop_tick().has_value());
  EXPECT_TRUE(sim.arm().brakes_engaged());
  EXPECT_TRUE(sim.arm().applied_torques().isZero());

  const Eigen::Vector3d at_stop = sim.true_tool_pose().translation();
  double farthest = 0.0;
  for (int i = 0; i < 3000; ++i) {
    sim.step();
    farthest = std::max(farthest, (sim.true_tool_pose().translation() - at_stop).norm());
    ASSERT_TRUE(sim.arm().applied_torques().isZero());
  }
  RecordProperty("motion_after_stop_um", std::to_string(farthest * kMicrometresPerMetre));
  EXPECT_LT(farthest, 0.2e-3);
  EXPECT_LT(sim.arm().state().motor_velocity.cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT(sim.arm().state().qd.cwiseAbs().maxCoeff(), 5e-3);
}

TEST(WithoutTheMonitor, AFrozenEncoderIsARunaway) {
  // The baseline that gives the campaign its meaning. Without this, a campaign would also pass
  // if the faults were simply harmless.
  stapes::test::verifies("SRS-SAF-023");
  double worst = 0.0;
  for (std::size_t joint = 0; joint < 3; ++joint) {
    KeyholeScenario scenario = without_limits(KeyholeScenario::reference(44));
    scenario.fault = Fault{FaultKind::frozen_motor_encoder, 800, joint, 0};
    const RunResult result = run_scenario(scenario, kRunTicks);
    EXPECT_EQ(result.cause, StopCause::none);
    worst = std::max(worst, result.max_radial_m);
  }
  RecordProperty("unmonitored_frozen_encoder_excursion_mm", std::to_string(worst * 1e3));
  EXPECT_GT(worst, 50e-3);
}

TEST(WithoutTheMonitor, ALongDelayIsUnstable) {
  stapes::test::verifies("SRS-SAF-023");
  KeyholeScenario scenario = without_limits(KeyholeScenario::reference(45));
  scenario.fault = Fault{FaultKind::comms_delay, 800, 0, 20};
  const RunResult result = run_scenario(scenario, kRunTicks);
  RecordProperty("unmonitored_delay_excursion_mm", std::to_string(result.max_radial_m * 1e3));
  EXPECT_GT(result.max_radial_m, 5e-3);
}

TEST(ControlLoop, StaysStableWithFiveTicksOfDelayEachWay) {
  // Margin against the limit the monitor enforces: the loop itself survives more delay than the
  // monitor will ever let it run with.
  stapes::test::verifies("SRS-CTL-011");
  KeyholeScenario scenario = without_limits(KeyholeScenario::reference(46));
  scenario.fault = Fault{FaultKind::comms_delay, 800, 0, 5};
  const RunResult result = run_scenario(scenario, kRunTicks);
  RecordProperty("delay_margin_max_radial_um",
                 std::to_string(result.max_radial_m * kMicrometresPerMetre));
  EXPECT_LT(result.max_radial_m, 0.2e-3);
}

}  // namespace
