// Runs that start from a rest pose instead of at the entry: the arm moves to the line in joint
// space, then feeds in. The monitor is the same throughout, and is never told which phase the
// run is in.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"
#include "stapes/sim/keyhole_simulation.hpp"
#include "stapes/task/keyhole.hpp"

namespace {

using stapes::faults::Fault;
using stapes::faults::FaultKind;
using stapes::kinematics::JointVector;
using stapes::safety::StopCause;
using stapes::sim::KeyholeScenario;
using stapes::sim::KeyholeSimulation;
using stapes::sim::with_approach;

constexpr double kMicrometresPerMetre = 1e6;
constexpr double kDesignTargetM = 0.2e-3;

// Ticks to the end of the feed, and half a second more to settle.
int full_run_ticks(const KeyholeSimulation& sim, const KeyholeScenario& scenario) {
  const double end_s = sim.feed_start_s() + sim.feed_duration_s() + 0.5;
  return static_cast<int>(std::ceil(end_s / scenario.safety.period_s));
}

// `with_approach` always sets one. Checked here so that every test reads it the same way.
stapes::sim::Approach& approach_of(KeyholeScenario& scenario) {
  if (!scenario.approach.has_value()) {
    throw std::logic_error("the scenario has no approach");
  }
  return *scenario.approach;
}

double guard_m(const KeyholeScenario& scenario) { return scenario.safety.corridor.entry_guard_m; }

TEST(Approach, StartsAtRestAndReachesTheLineBeforeTheGuard) {
  stapes::test::verifies("SRS-KEY-010");
  const KeyholeScenario scenario = with_approach(KeyholeScenario::reference(1));
  ASSERT_TRUE(scenario.approach.has_value());
  KeyholeSimulation sim(scenario);
  EXPECT_EQ(sim.arm().state().q, scenario.approach->rest_q);
  EXPECT_TRUE(sim.approaching());

  const stapes::task::KeyholeTrajectory& tunnel = scenario.trajectory;
  const Eigen::Vector3d rest_tip = sim.true_tool_pose().translation();
  EXPECT_LT(tunnel.depth_of(rest_tip), -0.1);
  EXPECT_GT(tunnel.radial_distance(rest_tip), 0.05);

  const double period_s = scenario.safety.period_s;
  const double move_end_s = scenario.approach->move_s;
  int settle_ticks = 0;
  JointVector held = JointVector::Zero();
  while (sim.approaching()) {
    sim.step();
    ASSERT_EQ(sim.monitor().cause(), StopCause::none);
    ASSERT_LT(tunnel.depth_of(sim.true_tool_pose().translation()), -guard_m(scenario));
    // After the move, the target holds at its end until the feed starts.
    if (static_cast<double>(sim.ticks()) * period_s > move_end_s + period_s) {
      if (settle_ticks++ == 0) {
        held = sim.target();
      }
      ASSERT_EQ(sim.target(), held);
      ASSERT_DOUBLE_EQ(sim.planned_depth_m(), -scenario.approach->standoff_m);
    }
  }
  EXPECT_NEAR(settle_ticks * period_s, scenario.approach->settle_s, 2.0 * period_s);
  // The feed is about to start: the tip waits on the line, at the standoff.
  const Eigen::Vector3d tip = sim.true_tool_pose().translation();
  EXPECT_NEAR(tunnel.depth_of(tip), -scenario.approach->standoff_m, kDesignTargetM);
  EXPECT_LT(tunnel.radial_distance(tip), kDesignTargetM);
  EXPECT_LT(stapes::task::axis_distance(sim.true_tool_pose(), tunnel.entry()), kDesignTargetM);
  // The plan is measured along the tunnel, so the feed starts from the standoff, not from zero.
  sim.step();
  EXPECT_NEAR(sim.planned_depth_m(), -scenario.approach->standoff_m, 1e-6);
}

TEST(Approach, AnApproachThroughTheGuardIsRefusedBeforeAnythingMoves) {
  stapes::test::verifies("SRS-KEY-010");
  KeyholeScenario scenario = with_approach(KeyholeScenario::reference(1));
  // A rest pose beside the tunnel and past the entry: on the way to the line the tip would be
  // inside the guard and off the line, which the monitor would stop.
  const stapes::kinematics::ArmModel& model = scenario.arm.model.arm();
  stapes::math::Pose beside = scenario.trajectory.pose_at(3e-3);
  beside.translation() += 30e-3 * scenario.trajectory.direction().unitOrthogonal();
  const stapes::kinematics::IkResult solved =
      stapes::kinematics::solve_ik(model, beside, approach_of(scenario).rest_q);
  ASSERT_TRUE(solved.converged());
  approach_of(scenario).rest_q = solved.q;
  EXPECT_THROW(KeyholeSimulation{scenario}, std::invalid_argument);
}

TEST(Approach, FromRestTheArmDrillsTheTunnelAcrossSeedsWithoutAStop) {
  stapes::test::verifies("SRS-KEY-011");
  constexpr int kSeeds = 4;
  double worst_speed_fraction = 0.0;
  double worst_radial = 0.0;
  double worst_axis = 0.0;
  double worst_depth = 0.0;
  for (int seed = 1; seed <= kSeeds; ++seed) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    const KeyholeScenario scenario =
        with_approach(KeyholeScenario::reference(static_cast<std::uint64_t>(seed)));
    const stapes::task::KeyholeTrajectory& tunnel = scenario.trajectory;
    KeyholeSimulation sim(scenario);
    const int ticks = full_run_ticks(sim, scenario);
    JointVector previous = sim.arm().state().q;
    for (int i = 0; i < ticks; ++i) {
      sim.step();
      const JointVector q = sim.arm().state().q;
      const JointVector speed = (q - previous).cwiseAbs() / scenario.safety.period_s;
      worst_speed_fraction =
          std::max(worst_speed_fraction,
                   (speed.array() / scenario.safety.max_velocity_rad_s.array()).maxCoeff());
      previous = q;
      const stapes::math::Pose tool = sim.true_tool_pose();
      if (tunnel.depth_of(tool.translation()) > -guard_m(scenario)) {
        worst_radial = std::max(worst_radial, tunnel.radial_distance(tool.translation()));
        worst_axis = std::max(worst_axis, stapes::task::axis_distance(tool, tunnel.entry()));
      }
    }
    EXPECT_EQ(sim.monitor().cause(), StopCause::none);
    EXPECT_NEAR(sim.planned_depth_m(), tunnel.length_m(), 1e-12);
    worst_depth =
        std::max(worst_depth,
                 std::abs(tunnel.depth_of(sim.true_tool_pose().translation()) - tunnel.length_m()));
  }
  RecordProperty("approach_max_radial_um", std::to_string(worst_radial * kMicrometresPerMetre));
  RecordProperty("approach_max_axis_um", std::to_string(worst_axis * kMicrometresPerMetre));
  RecordProperty("approach_final_depth_error_um",
                 std::to_string(worst_depth * kMicrometresPerMetre));
  RecordProperty("approach_peak_speed_fraction_of_limit", std::to_string(worst_speed_fraction));
  EXPECT_LT(worst_speed_fraction, 1.0);
  EXPECT_LT(worst_radial, kDesignTargetM);
  EXPECT_LT(worst_axis, kDesignTargetM);
  EXPECT_LT(worst_depth, kDesignTargetM);
}

TEST(Approach, EveryFaultDuringTheApproachIsStoppedBeforeTheGuard) {
  stapes::test::verifies("SRS-SAF-025");
  constexpr std::array<FaultKind, 5> kKinds{
      FaultKind::frozen_motor_encoder, FaultKind::frozen_link_encoder, FaultKind::stuck_actuator,
      FaultKind::frozen_tracker,       FaultKind::comms_delay,
  };
  constexpr int kDelayTicks = 20;
  std::int64_t worst_latency = 0;
  int runs = 0;
  for (const FaultKind kind : kKinds) {
    for (const std::size_t joint : {1U, 4U}) {
      // Early, while the arm accelerates, and late, as it slows down to reach the line.
      for (const double fraction : {0.2, 0.8}) {
        KeyholeScenario scenario = with_approach(KeyholeScenario::reference(31));
        const auto start = static_cast<std::int64_t>(fraction * approach_of(scenario).move_s /
                                                     scenario.safety.period_s);
        SCOPED_TRACE("kind " + std::to_string(static_cast<int>(kind)) + " joint " +
                     std::to_string(joint) + " start " + std::to_string(start));
        scenario.fault = Fault{kind, start, joint, kDelayTicks};
        KeyholeSimulation sim(scenario);
        while (sim.approaching() && !sim.stop_tick().has_value()) {
          sim.step();
          ASSERT_LT(scenario.trajectory.depth_of(sim.true_tool_pose().translation()),
                    -guard_m(scenario));
        }
        ++runs;
        ASSERT_TRUE(sim.stop_tick().has_value());
        worst_latency = std::max(worst_latency, *sim.stop_tick() - start);
      }
    }
  }
  RecordProperty("approach_fault_runs", runs);
  RecordProperty("approach_fault_worst_latency_ms", std::to_string(worst_latency));
}

}  // namespace
