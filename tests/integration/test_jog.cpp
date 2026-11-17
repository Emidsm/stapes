// An operator in place of the plan: free jogging, alignment with the line, a feed set by the
// operator, and the same monitor throughout. Inputs here are scripted, as a recording replays.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/base/state_hash.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/sim/jog_simulation.hpp"
#include "stapes/task/keyhole.hpp"

namespace {

using stapes::faults::Fault;
using stapes::faults::FaultKind;
using stapes::safety::StopCause;
using stapes::sim::AlignRefusal;
using stapes::sim::JogMode;
using stapes::sim::JogSimulation;
using stapes::sim::KeyholeScenario;
using stapes::sim::OperatorInput;
using stapes::sim::with_approach;

constexpr double kDesignTargetM = 0.2e-3;
constexpr int kTicksPerSecond = 1000;

KeyholeScenario jog_scenario(FaultKind fault = FaultKind::none) {
  KeyholeScenario scenario = with_approach(KeyholeScenario::reference(1));
  scenario.fault = Fault{fault, 0, 1, 0};
  return scenario;
}

// `with_approach` always sets one. Checked here so that every test reads it the same way.
double standoff_m(const KeyholeScenario& scenario) {
  if (!scenario.approach.has_value()) {
    throw std::logic_error("the scenario has no approach");
  }
  return scenario.approach->standoff_m;
}

Eigen::Vector3d tip(const JogSimulation& sim) { return sim.true_tool_pose().translation(); }

// A horizontal direction across the tunnel, so that moving along it changes no depth.
Eigen::Vector3d across(const JogSimulation& sim) {
  return sim.trajectory().direction().cross(Eigen::Vector3d::UnitZ()).normalized();
}

void hold(JogSimulation& sim, const OperatorInput& input, int ticks) {
  for (int i = 0; i < ticks; ++i) {
    sim.step(input);
  }
}

// One press of a button: down for a tick, then up.
void press_align(JogSimulation& sim) {
  OperatorInput input;
  input.align = true;
  sim.step(input);
  sim.step(OperatorInput{});
}

void align_and_settle(JogSimulation& sim) {
  press_align(sim);
  int guard = 0;
  while (sim.mode() != JogMode::on_line && guard++ < 20 * kTicksPerSecond) {
    sim.step(OperatorInput{});
  }
  hold(sim, OperatorInput{}, kTicksPerSecond / 2);
}

// Feeds at `feed` until the planned depth passes `depth_m` in the direction of the feed.
void feed_to(JogSimulation& sim, double feed, double depth_m) {
  OperatorInput input;
  input.feed = feed;
  int guard = 0;
  while ((feed > 0 ? sim.planned_depth_m() < depth_m : sim.planned_depth_m() > depth_m) &&
         guard++ < 20 * kTicksPerSecond) {
    sim.step(input);
  }
}

TEST(Jog, NeedsARestPoseToStartFrom) {
  stapes::test::verifies("SRS-JOG-001");
  EXPECT_THROW(JogSimulation(KeyholeScenario::reference(1)), std::invalid_argument);
}

TEST(Jog, TheTipFollowsTheStickWithinTheSpeedLimitAndKeepsTheToolDirection) {
  stapes::test::verifies("SRS-JOG-001");
  JogSimulation sim(jog_scenario());
  EXPECT_EQ(sim.mode(), JogMode::free);
  const Eigen::Vector3d axis_before = stapes::task::tool_axis(sim.true_tool_pose());
  const Eigen::Vector3d start = tip(sim);

  OperatorInput input;
  input.tip_velocity_m_s = 0.02 * across(sim);
  hold(sim, input, 2 * kTicksPerSecond);
  // Ramping up to 20 mm/s at 0.025 m/s^2 takes 0.8 s and costs 8 mm: 32 mm in two seconds.
  const double moved_m = (tip(sim) - start).dot(across(sim));
  EXPECT_NEAR(moved_m, 0.032, 1e-3);

  // Far beyond the limit, and the other way: the tip goes no faster than the limit.
  input.tip_velocity_m_s = -1.0 * across(sim);
  hold(sim, input, 3 * kTicksPerSecond);
  const Eigen::Vector3d before_fast = tip(sim);
  hold(sim, input, kTicksPerSecond);
  EXPECT_LE((tip(sim) - before_fast).norm(), JogSimulation::kMaxTipSpeedMS + 1e-3);
  EXPECT_GT((tip(sim) - before_fast).norm(), 0.5 * JogSimulation::kMaxTipSpeedMS);

  // Let go: the tip comes to rest.
  hold(sim, OperatorInput{}, 2 * kTicksPerSecond);
  const Eigen::Vector3d resting = tip(sim);
  hold(sim, OperatorInput{}, kTicksPerSecond / 10);
  EXPECT_LT((tip(sim) - resting).norm(), 0.1e-3);

  const Eigen::Vector3d axis_after = stapes::task::tool_axis(sim.true_tool_pose());
  EXPECT_LT(std::acos(std::clamp(axis_before.dot(axis_after), -1.0, 1.0)), 1e-3);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

// The tip the controller is being told to reach, from the target configuration.
Eigen::Vector3d target_tip(const JogSimulation& sim, const KeyholeScenario& scenario) {
  return stapes::kinematics::forward_kinematics(scenario.arm.model.arm(), sim.target())
      .translation();
}

TEST(Jog, TheCommandedTipNeverExceedsTheSpeedLimit) {
  stapes::test::verifies("SRS-JOG-001");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  OperatorInput input;
  input.tip_velocity_m_s = Eigen::Vector3d(-1.0, 0.0, 0.0);
  double fastest_m_s = 0.0;
  Eigen::Vector3d before = target_tip(sim, scenario);
  for (int i = 0; i < 4 * kTicksPerSecond; ++i) {
    sim.step(input);
    const Eigen::Vector3d now = target_tip(sim, scenario);
    fastest_m_s = std::max(fastest_m_s, (now - before).norm() * kTicksPerSecond);
    before = now;
  }
  EXPECT_LE(fastest_m_s, JogSimulation::kMaxTipSpeedMS * (1.0 + 1e-6));
  EXPECT_GT(fastest_m_s, 0.9 * JogSimulation::kMaxTipSpeedMS);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(Jog, NearASingularityTheTipSlowsInsteadOfTheJointsRacing) {
  stapes::test::verifies("SRS-JOG-001");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  OperatorInput input;
  input.tip_velocity_m_s = -1.0 * across(sim);
  double busiest = 0.0;
  stapes::kinematics::JointVector before = sim.target();
  for (int i = 0; i < 8 * kTicksPerSecond; ++i) {
    sim.step(input);
    const double load = ((sim.target() - before).array().abs() /
                         (scenario.safety.max_velocity_rad_s.array() / kTicksPerSecond))
                            .maxCoeff();
    busiest = std::max(busiest, load);
    before = sim.target();
  }
  RecordProperty("jog_peak_joint_speed_fraction", std::to_string(busiest));
  // The second solve after slowing down is not exactly linear in the step: allow 1 %.
  EXPECT_LE(busiest, JogSimulation::kJointSpeedFraction * 1.01);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(Jog, ANonFiniteCommandWhileMovingSlowsDownAsReleasingWould) {
  stapes::test::verifies("SRS-JOG-006");
  const auto run = [](double glitch) {
    JogSimulation sim(jog_scenario());
    OperatorInput input;
    input.tip_velocity_m_s = 0.03 * across(sim);
    hold(sim, input, kTicksPerSecond);
    OperatorInput odd;
    odd.tip_velocity_m_s = Eigen::Vector3d::Constant(glitch);
    sim.step(odd);
    hold(sim, input, 100);
    stapes::base::StateHash hash;
    sim.add_to_hash(hash);
    return hash.value();
  };
  EXPECT_EQ(run(std::numeric_limits<double>::quiet_NaN()), run(0.0));
}

TEST(Jog, ANonFiniteCommandIsNoMotion) {
  stapes::test::verifies("SRS-JOG-006");
  JogSimulation still(jog_scenario());
  JogSimulation fed_nan(jog_scenario());
  OperatorInput bad;
  bad.tip_velocity_m_s = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  bad.feed = std::numeric_limits<double>::infinity();
  hold(still, OperatorInput{}, 200);
  hold(fed_nan, bad, 200);
  stapes::base::StateHash a;
  stapes::base::StateHash b;
  still.add_to_hash(a);
  fed_nan.add_to_hash(b);
  EXPECT_EQ(a.value(), b.value());
  EXPECT_EQ(fed_nan.monitor().cause(), StopCause::none);
}

TEST(Jog, AlignmentBringsTheTipOntoTheLineAtTheStandoff) {
  stapes::test::verifies("SRS-JOG-002");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  // Start from somewhere the operator has put the arm, not from the planned rest pose.
  OperatorInput input;
  input.tip_velocity_m_s = 0.03 * across(sim) + Eigen::Vector3d(0.0, 0.0, 0.01);
  hold(sim, input, kTicksPerSecond);

  // A planned move starts from rest, so the tip has to stop first.
  sim.step(OperatorInput{});
  press_align(sim);
  EXPECT_EQ(sim.mode(), JogMode::free);
  EXPECT_EQ(sim.align_refusal(), AlignRefusal::still_moving);
  hold(sim, OperatorInput{}, 2 * kTicksPerSecond);

  press_align(sim);
  EXPECT_EQ(sim.mode(), JogMode::aligning);
  EXPECT_EQ(sim.align_refusal(), AlignRefusal::none);
  align_and_settle(sim);
  ASSERT_EQ(sim.mode(), JogMode::on_line);
  EXPECT_LT(sim.trajectory().radial_distance(tip(sim)), kDesignTargetM);
  EXPECT_NEAR(sim.trajectory().depth_of(tip(sim)), -standoff_m(scenario), kDesignTargetM);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(Jog, TheOperatorSetsTheFeedAndTheArmHoldsTheLine) {
  stapes::test::verifies("SRS-JOG-003");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  align_and_settle(sim);

  // The stick does nothing on the line.
  OperatorInput sideways;
  sideways.tip_velocity_m_s = 0.05 * across(sim);
  hold(sim, sideways, kTicksPerSecond / 2);
  EXPECT_LT(sim.trajectory().radial_distance(tip(sim)), kDesignTargetM);

  const double length_m = sim.trajectory().length_m();
  const double guard_m = scenario.safety.corridor.entry_guard_m;
  double worst_radial_m = 0.0;
  double fastest_m_s = 0.0;
  double previous_depth_m = sim.planned_depth_m();
  OperatorInput input;
  input.feed = 3.0;  // More than full: no faster than the plan.
  int guard = 0;
  while (sim.planned_depth_m() < length_m && guard++ < 20 * kTicksPerSecond) {
    sim.step(input);
    if (sim.trajectory().depth_of(tip(sim)) > -guard_m) {
      worst_radial_m = std::max(worst_radial_m, sim.trajectory().radial_distance(tip(sim)));
    }
    fastest_m_s = std::max(fastest_m_s, (sim.planned_depth_m() - previous_depth_m) * 1000.0);
    previous_depth_m = sim.planned_depth_m();
  }
  hold(sim, OperatorInput{}, kTicksPerSecond / 2);
  RecordProperty("jog_max_radial_um", std::to_string(worst_radial_m * 1e6));
  EXPECT_LT(worst_radial_m, kDesignTargetM);
  EXPECT_LE(fastest_m_s, scenario.feed_speed_m_s * (1.0 + 1e-9));
  EXPECT_NEAR(sim.trajectory().depth_of(tip(sim)), length_m, kDesignTargetM);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);

  // Further is not possible: the feed stops at the target.
  hold(sim, input, kTicksPerSecond);
  EXPECT_NEAR(sim.planned_depth_m(), length_m, 1e-12);

  // Back out, and let go of the line at the standoff.
  feed_to(sim, -1.0, -standoff_m(scenario));
  hold(sim, OperatorInput{}, kTicksPerSecond / 2);
  press_align(sim);
  EXPECT_EQ(sim.mode(), JogMode::free);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(Jog, HoldingTheButtonDownActsOnce) {
  stapes::test::verifies("SRS-JOG-002");
  JogSimulation sim(jog_scenario());
  OperatorInput input;
  input.align = true;
  hold(sim, input, 15 * kTicksPerSecond);
  EXPECT_EQ(sim.mode(), JogMode::on_line);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(Jog, LettingGoOfTheLineAwayFromTheStandoffIsRefused) {
  stapes::test::verifies("SRS-JOG-003");
  JogSimulation sim(jog_scenario());
  align_and_settle(sim);
  feed_to(sim, 1.0, 3e-3);
  press_align(sim);
  EXPECT_EQ(sim.mode(), JogMode::on_line);
  EXPECT_EQ(sim.align_refusal(), AlignRefusal::not_at_standoff);
}

TEST(Jog, TheMonitorStopsATipJoggedIntoTheBoneOffTheLine) {
  stapes::test::verifies("SRS-JOG-004");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  align_and_settle(sim);
  press_align(sim);
  ASSERT_EQ(sim.mode(), JogMode::free);

  // Five millimetres to the side, then straight at the bone.
  OperatorInput input;
  input.tip_velocity_m_s = 0.01 * across(sim);
  hold(sim, input, 500);
  hold(sim, OperatorInput{}, kTicksPerSecond);
  input.tip_velocity_m_s = 0.01 * sim.trajectory().direction();
  int guard = 0;
  while (!sim.stop_tick().has_value() && guard++ < 2 * kTicksPerSecond) {
    sim.step(input);
  }
  ASSERT_TRUE(sim.stop_tick().has_value());
  EXPECT_EQ(sim.monitor().cause(), StopCause::corridor);
  // Stopped inside the guard, before the tip reached the bone.
  hold(sim, input, kTicksPerSecond / 2);
  EXPECT_LT(sim.trajectory().depth_of(tip(sim)), 0.0);
  EXPECT_GT(sim.trajectory().depth_of(tip(sim)), -scenario.safety.corridor.entry_guard_m - 0.5e-3);
}

TEST(Jog, AlignmentThatWouldCutInsideTheGuardIsRefused) {
  stapes::test::verifies("SRS-JOG-002");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  align_and_settle(sim);
  press_align(sim);
  // Along the line by hand, into the guard. The free mode holds the tool's direction, so the
  // tip stays on the line and the monitor lets it through.
  OperatorInput input;
  input.tip_velocity_m_s = 0.01 * sim.trajectory().direction();
  while (sim.trajectory().depth_of(tip(sim)) < -1.5e-3 && !sim.stop_tick().has_value()) {
    sim.step(input);
  }
  hold(sim, OperatorInput{}, kTicksPerSecond);
  ASSERT_EQ(sim.monitor().cause(), StopCause::none);

  press_align(sim);
  EXPECT_EQ(sim.mode(), JogMode::free);
  EXPECT_EQ(sim.align_refusal(), AlignRefusal::enters_guard);
}

TEST(Jog, AFaultStartedByTheOperatorIsContained) {
  stapes::test::verifies("SRS-JOG-005");
  JogSimulation sim(jog_scenario(FaultKind::frozen_motor_encoder));
  align_and_settle(sim);
  feed_to(sim, 1.0, 3e-3);
  EXPECT_FALSE(sim.fault_active());

  OperatorInput input;
  input.feed = 1.0;
  input.fault = true;
  sim.step(input);
  ASSERT_TRUE(sim.fault_start_tick().has_value());
  EXPECT_TRUE(sim.fault_active());
  int guard = 0;
  while (!sim.stop_tick().has_value() && guard++ < kTicksPerSecond) {
    sim.step(input);
  }
  ASSERT_TRUE(sim.stop_tick().has_value());
  EXPECT_NE(sim.monitor().cause(), StopCause::none);
  hold(sim, input, kTicksPerSecond / 2);
  EXPECT_LT(sim.trajectory().radial_distance(tip(sim)), 0.5e-3);
}

TEST(Jog, TheFaultButtonDoesNothingWithoutAFault) {
  stapes::test::verifies("SRS-JOG-005");
  JogSimulation sim(jog_scenario());
  OperatorInput input;
  input.fault = true;
  hold(sim, input, 10);
  EXPECT_FALSE(sim.fault_active());
}

void press_joint_mode(JogSimulation& sim) {
  OperatorInput input;
  input.joint_mode = true;
  sim.step(input);
  sim.step(OperatorInput{});
}

OperatorInput turning(int joint, double rate) {
  OperatorInput input;
  input.joint = joint;
  input.joint_rate = rate;
  return input;
}

TEST(JointJog, OnlyTheChosenJointTurnsWithinItsLimits) {
  stapes::test::verifies("SRS-JOG-007");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  press_joint_mode(sim);
  ASSERT_EQ(sim.mode(), JogMode::joint);

  const stapes::kinematics::JointVector start = sim.target();
  stapes::kinematics::JointVector before = start;
  double fastest = 0.0;
  double steepest = 0.0;
  double previous_speed = 0.0;
  for (int i = 0; i < 3 * kTicksPerSecond; ++i) {
    sim.step(turning(2, 1.0));
    const double speed = (sim.target()(2) - before(2)) * kTicksPerSecond;
    fastest = std::max(fastest, std::abs(speed));
    steepest = std::max(steepest, std::abs(speed - previous_speed) * kTicksPerSecond);
    previous_speed = speed;
    before = sim.target();
  }
  const double top = JogSimulation::kJointSpeedFraction * scenario.safety.max_velocity_rad_s(2);
  EXPECT_NEAR(fastest, top, 1e-9);
  EXPECT_LE(steepest, JogSimulation::kMaxJointAccelerationRadS2 * (1.0 + 1e-6));
  // Two seconds at the top speed after a two-second ramp: 0.1 + 0.1 rad.
  EXPECT_NEAR(sim.target()(2) - start(2), 0.2, 1e-3);
  for (int joint = 0; joint < 6; ++joint) {
    if (joint != 2) {
      EXPECT_EQ(sim.target()(joint), start(joint)) << "joint " << joint;
    }
  }
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(JointJog, EveryJointTurnsBothWaysAtFullSpeedWithoutAStop) {
  stapes::test::verifies("SRS-JOG-007");
  for (int joint = 0; joint < 6; ++joint) {
    for (const double rate : {1.0, -1.0}) {
      JogSimulation sim(jog_scenario());
      press_joint_mode(sim);
      hold(sim, turning(joint, rate), 3 * kTicksPerSecond);
      hold(sim, OperatorInput{}, 3 * kTicksPerSecond);
      EXPECT_EQ(sim.monitor().cause(), StopCause::none) << "joint " << joint << " rate " << rate;
    }
  }
}

TEST(JointJog, ChangingJointsSlowsTheFirstOneDown) {
  stapes::test::verifies("SRS-JOG-007");
  JogSimulation sim(jog_scenario());
  press_joint_mode(sim);
  hold(sim, turning(0, 1.0), 2 * kTicksPerSecond);
  double previous = sim.target()(0);
  double previous_speed = JogSimulation::kJointSpeedFraction * 0.2;
  double steepest = 0.0;
  for (int i = 0; i < 3 * kTicksPerSecond; ++i) {
    sim.step(turning(1, 1.0));
    const double speed = (sim.target()(0) - previous) * kTicksPerSecond;
    steepest = std::max(steepest, std::abs(speed - previous_speed) * kTicksPerSecond);
    previous_speed = speed;
    previous = sim.target()(0);
  }
  EXPECT_LE(steepest, JogSimulation::kMaxJointAccelerationRadS2 * (1.0 + 1e-6));
  EXPECT_EQ(previous_speed, 0.0);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(JointJog, AJointStopsAtItsSoftLimitWithoutAStop) {
  stapes::test::verifies("SRS-JOG-007");
  const KeyholeScenario scenario = jog_scenario();
  JogSimulation sim(scenario);
  press_joint_mode(sim);
  // Joint 5, the wrist roll, has the least range to the side it turns to here.
  hold(sim, turning(5, -1.0), 60 * kTicksPerSecond);
  const auto& joint = scenario.safety.arm.joints()[5];
  const double margin = scenario.safety.joint_limit_margin_rad + JogSimulation::kSoftLimitMarginRad;
  EXPECT_NEAR(sim.target()(5), joint.lower_rad + margin, 1e-3);
  EXPECT_GE(sim.target()(5), joint.lower_rad + margin);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(JointJog, SwitchingModesWaitsForTheArmToStop) {
  stapes::test::verifies("SRS-JOG-007");
  JogSimulation sim(jog_scenario());
  press_joint_mode(sim);
  hold(sim, turning(1, 1.0), kTicksPerSecond);
  OperatorInput toggle = turning(1, 1.0);
  toggle.joint_mode = true;
  sim.step(toggle);
  EXPECT_EQ(sim.mode(), JogMode::joint);
  EXPECT_EQ(sim.align_refusal(), AlignRefusal::still_moving);

  hold(sim, OperatorInput{}, 2 * kTicksPerSecond);
  press_joint_mode(sim);
  EXPECT_EQ(sim.mode(), JogMode::free);
  // Free mode carries on from where the joints left the tip.
  const Eigen::Vector3d start = tip(sim);
  OperatorInput input;
  input.tip_velocity_m_s = Eigen::Vector3d(0.0, 0.0, 0.02);
  hold(sim, input, kTicksPerSecond);
  EXPECT_GT(tip(sim).z() - start.z(), 5e-3);
  EXPECT_EQ(sim.monitor().cause(), StopCause::none);
}

TEST(JointJog, AnUnknownJointOrANonFiniteRateMovesNothing) {
  stapes::test::verifies("SRS-JOG-007");
  JogSimulation sim(jog_scenario());
  press_joint_mode(sim);
  const stapes::kinematics::JointVector start = sim.target();
  hold(sim, turning(6, 1.0), 200);
  hold(sim, turning(-1, 1.0), 200);
  hold(sim, turning(0, std::numeric_limits<double>::quiet_NaN()), 200);
  EXPECT_EQ(sim.target(), start);
}

// A session of every kind of input, as a function of the tick.
OperatorInput scripted(std::int64_t tick, const Eigen::Vector3d& across_dir) {
  OperatorInput input;
  if (tick < 800) {
    input.tip_velocity_m_s = 0.02 * std::sin(static_cast<double>(tick) * 0.004) * across_dir;
  } else if (tick < 802) {
    input.align = true;
  } else if (tick > 3000) {
    input.feed = 0.6;
    input.fault = tick > 4000;
  }
  return input;
}

std::uint64_t session_hash(double scale) {
  JogSimulation sim(jog_scenario(FaultKind::stuck_actuator));
  const Eigen::Vector3d dir = scale * across(sim);
  stapes::base::StateHash hash;
  for (std::int64_t tick = 0; tick < 5000; ++tick) {
    sim.step(scripted(tick, dir));
    sim.add_to_hash(hash);
  }
  return hash.value();
}

TEST(Jog, ARecordedSessionReplaysExactly) {
  stapes::test::verifies("SRS-JOG-006");
  EXPECT_EQ(session_hash(1.0), session_hash(1.0));
  EXPECT_NE(session_hash(1.0), session_hash(0.5));
}

}  // namespace
