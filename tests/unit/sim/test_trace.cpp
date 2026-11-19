#include <cstddef>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/sim/keyhole_simulation.hpp"
#include "stapes/sim/trace.hpp"
#include "stapes/task/keyhole.hpp"

namespace {

using stapes::faults::Fault;
using stapes::faults::FaultKind;
using stapes::safety::StopCause;
using stapes::sim::KeyholeScenario;
using stapes::sim::KeyholeSimulation;
using stapes::sim::TraceRecorder;
using stapes::sim::TraceSample;

TEST(TraceRecorder, RejectsZeroCapacity) {
  stapes::test::verifies("SRS-TRC-002");
  EXPECT_THROW(TraceRecorder{0}, std::invalid_argument);
}

TEST(TraceRecorder, StartsEmptyWithItsCapacityReserved) {
  stapes::test::verifies("SRS-TRC-002");
  const TraceRecorder recorder(64);
  EXPECT_TRUE(recorder.samples().empty());
  EXPECT_EQ(recorder.capacity(), 64U);
  EXPECT_EQ(recorder.dropped(), 0U);
}

TEST(TraceRecorder, KeepsOneSamplePerCallInOrder) {
  stapes::test::verifies("SRS-TRC-001");
  KeyholeSimulation sim(KeyholeScenario::reference(1));
  TraceRecorder recorder(100);
  for (int i = 0; i < 100; ++i) {
    sim.step();
    recorder.record(sim);
  }
  ASSERT_EQ(recorder.samples().size(), 100U);
  for (std::size_t i = 0; i < recorder.samples().size(); ++i) {
    EXPECT_EQ(recorder.samples()[i].tick, static_cast<std::int64_t>(i) + 1);
  }
}

TEST(TraceRecorder, ASampleHoldsTheTrueStateAtThatTick) {
  stapes::test::verifies("SRS-TRC-001");
  KeyholeSimulation sim(KeyholeScenario::reference(2));
  TraceRecorder recorder(1);
  for (int i = 0; i < 700; ++i) {
    sim.step();
  }
  recorder.record(sim);
  const TraceSample& sample = recorder.samples().front();
  const stapes::math::Pose tool = sim.true_tool_pose();

  EXPECT_EQ(sample.tick, 700);
  EXPECT_EQ(sample.planned_depth_m, sim.planned_depth_m());
  EXPECT_GT(sample.planned_depth_m, 0.0);  // The feed is under way, so the values are not trivial.
  EXPECT_EQ(sample.q, sim.arm().state().q);
  EXPECT_EQ(sample.target, sim.target());
  EXPECT_TRUE(sample.tool.isApprox(tool, 0.0));
  EXPECT_EQ(sample.radial_m, sim.trajectory().radial_distance(tool.translation()));
  EXPECT_EQ(sample.axis_offset_m, stapes::task::axis_distance(tool, sim.trajectory().entry()));
  EXPECT_EQ(sample.depth_m, sim.trajectory().depth_of(tool.translation()));
  EXPECT_GT(sample.depth_m, 1e-3);
  EXPECT_EQ(sample.cause, StopCause::none);
  EXPECT_FALSE(sample.fault_active);
  EXPECT_FALSE(sample.brakes_engaged);
  EXPECT_FALSE(sample.approaching);
}

TEST(TraceRecorder, MarksTheSamplesTakenDuringAnApproach) {
  stapes::test::verifies("SRS-TRC-001");
  KeyholeSimulation sim(stapes::sim::with_approach(KeyholeScenario::reference(2)));
  TraceRecorder recorder(2);
  sim.step();
  recorder.record(sim);
  EXPECT_TRUE(recorder.samples().front().approaching);
  // Before the entry, so the planned depth is negative: the tip waits at the standoff.
  EXPECT_LT(recorder.samples().front().planned_depth_m, 0.0);
  while (sim.approaching()) {
    sim.step();
  }
  recorder.record(sim);
  EXPECT_FALSE(recorder.samples().back().approaching);
}

TEST(TraceRecorder, ShowsTheFaultStartingAndTheStopThatFollows) {
  stapes::test::verifies("SRS-TRC-001");
  constexpr std::int64_t kFaultStart = 300;
  KeyholeScenario scenario = KeyholeScenario::reference(3);
  scenario.fault = Fault{FaultKind::comms_delay, kFaultStart, 0, 50};
  KeyholeSimulation sim(scenario);
  TraceRecorder recorder(400);
  for (int i = 0; i < 400; ++i) {
    sim.step();
    recorder.record(sim);
  }
  for (const TraceSample& sample : recorder.samples()) {
    SCOPED_TRACE("tick " + std::to_string(sample.tick));
    EXPECT_EQ(sample.fault_active, sample.tick >= kFaultStart);
    // The monitor sees the late sample during tick 300, so the state after that tick is stopped.
    const bool stopped = sample.tick > kFaultStart;
    EXPECT_EQ(sample.cause, stopped ? StopCause::stale_data : StopCause::none);
    EXPECT_EQ(sample.brakes_engaged, stopped);
  }
}

TEST(TraceRecorder, AFullRecorderDropsAndCountsInsteadOfGrowing) {
  stapes::test::verifies("SRS-TRC-002");
  KeyholeSimulation sim(KeyholeScenario::reference(1));
  TraceRecorder recorder(10);
  for (int i = 0; i < 25; ++i) {
    sim.step();
    recorder.record(sim);
  }
  EXPECT_EQ(recorder.samples().size(), 10U);
  EXPECT_EQ(recorder.capacity(), 10U);
  EXPECT_EQ(recorder.dropped(), 15U);
  // The first samples are the ones kept. Nothing was overwritten.
  EXPECT_EQ(recorder.samples().front().tick, 1);
  EXPECT_EQ(recorder.samples().back().tick, 10);
}

TEST(TraceRecorder, ClearForgetsSamplesAndKeepsTheStorage) {
  stapes::test::verifies("SRS-TRC-002");
  KeyholeSimulation sim(KeyholeScenario::reference(1));
  TraceRecorder recorder(4);
  for (int i = 0; i < 6; ++i) {
    sim.step();
    recorder.record(sim);
  }
  recorder.clear();
  EXPECT_TRUE(recorder.samples().empty());
  EXPECT_EQ(recorder.dropped(), 0U);
  EXPECT_EQ(recorder.capacity(), 4U);
  recorder.record(sim);
  EXPECT_EQ(recorder.samples().size(), 1U);
  EXPECT_EQ(recorder.samples().front().tick, 6);
}

TEST(TraceRecorder, RecordingDoesNotChangeTheRun) {
  // An observer that altered what it observed would make every recorded result suspect.
  stapes::test::verifies("SRS-TRC-003");
  KeyholeScenario scenario = KeyholeScenario::reference(4);
  scenario.fault = Fault{FaultKind::frozen_motor_encoder, 500, 1, 0};
  KeyholeSimulation plain(scenario);
  KeyholeSimulation recorded(scenario);
  TraceRecorder recorder(1500);
  stapes::base::StateHash plain_hash;
  stapes::base::StateHash recorded_hash;
  for (int i = 0; i < 1500; ++i) {
    plain.step();
    plain.add_to_hash(plain_hash);
    recorded.step();
    recorder.record(recorded);
    recorded.add_to_hash(recorded_hash);
  }
  EXPECT_EQ(plain_hash.value(), recorded_hash.value());
  EXPECT_TRUE(recorded.stop_tick().has_value());
}

TEST(WithoutMonitorLimits, NoLimitIsLeftWithinReach) {
  stapes::test::verifies("SRS-SAF-023");
  const KeyholeScenario reference = KeyholeScenario::reference(1);
  const KeyholeScenario open = stapes::sim::without_monitor_limits(reference);
  const stapes::safety::SafetyConfig& limits = open.safety;
  constexpr double kFar = 1e6;
  for (const stapes::kinematics::RevoluteJoint& joint : limits.arm.joints()) {
    EXPECT_LT(joint.lower_rad, -kFar);
    EXPECT_GT(joint.upper_rad, kFar);
  }
  EXPECT_GT(limits.max_velocity_rad_s.minCoeff(), kFar);
  EXPECT_GT(limits.max_tool_force_n, kFar);
  EXPECT_GT(limits.corridor.radius_m, kFar);
  EXPECT_GT(limits.corridor.overshoot_margin_m, kFar);
  EXPECT_GT(limits.max_tracker_disagreement_m, kFar);
  EXPECT_GT(limits.max_encoder_disagreement_rad.minCoeff(), kFar);
  EXPECT_GT(limits.max_tracking_error_rad.minCoeff(), kFar);
  EXPECT_GT(limits.max_sample_age_ticks, 100000);
  // The result is still a configuration the monitor accepts, and nothing else has changed.
  EXPECT_NO_THROW(stapes::safety::SafetyMonitor{limits});
  EXPECT_EQ(open.feed_speed_m_s, reference.feed_speed_m_s);
  EXPECT_EQ(open.trajectory.length_m(), reference.trajectory.length_m());
  EXPECT_EQ(limits.debounce_checks, reference.safety.debounce_checks);
}

}  // namespace
