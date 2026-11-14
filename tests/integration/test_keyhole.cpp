#include <algorithm>
#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "closed_loop_workload.hpp"
#include "fault_scenarios.hpp"
#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"
#include "stapes/task/keyhole.hpp"

namespace {

using stapes::kinematics::JointVector;
using stapes::math::Wrench;
using stapes::sim::KeyholeScenario;
using stapes::sim::KeyholeSimulation;
using stapes::test::kRunTicks;
using stapes::test::run_scenario;

constexpr double kMicrometresPerMetre = 1e6;

TEST(KeyholeDrilling, FollowsThePlannedLineAcrossSeedsWithoutAFalseStop) {
  stapes::test::verifies("SRS-KEY-006,SRS-SAF-018");
  constexpr int kSeeds = 20;
  double worst_radial = 0.0;
  double worst_axis = 0.0;
  double worst_depth = 0.0;
  for (int seed = 1; seed <= kSeeds; ++seed) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    const KeyholeScenario scenario = KeyholeScenario::reference(static_cast<std::uint64_t>(seed));
    const stapes::test::RunResult result = run_scenario(scenario, kRunTicks);
    EXPECT_EQ(result.cause, stapes::safety::StopCause::none);
    worst_radial = std::max(worst_radial, result.max_radial_m);
    worst_axis = std::max(worst_axis, result.max_axis_offset_m);
    worst_depth =
        std::max(worst_depth, std::abs(result.final_depth_m - scenario.trajectory.length_m()));
  }
  RecordProperty("nominal_max_radial_um", std::to_string(worst_radial * kMicrometresPerMetre));
  RecordProperty("nominal_max_axis_um", std::to_string(worst_axis * kMicrometresPerMetre));
  RecordProperty("nominal_final_depth_error_um",
                 std::to_string(worst_depth * kMicrometresPerMetre));
  EXPECT_LT(worst_radial, 0.2e-3);
  EXPECT_LT(worst_axis, 0.2e-3);
  EXPECT_LT(worst_depth, 0.2e-3);
}

TEST(KeyholeDrilling, AHealthyRunKeepsAMarginToEveryMonitorLimit) {
  // The monitor's limits were chosen from what a healthy run looks like through the monitor's
  // own sensors. This measures that again on every build, so a change to a sensor or a gain that
  // eats the margin shows up here before it shows up as a false stop.
  stapes::test::verifies("SRS-SAF-018");
  constexpr int kSeeds = 60;
  double tracker_against_links = 0.0;
  double tracked_tip_off_line = 0.0;
  double tracked_shaft_at_entry = 0.0;
  double motor_against_link = 0.0;
  double link_against_target = 0.0;
  double tool_force = 0.0;
  for (int seed = 1; seed <= kSeeds; ++seed) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    const KeyholeScenario scenario = KeyholeScenario::reference(static_cast<std::uint64_t>(seed));
    KeyholeSimulation sim(scenario);
    for (int i = 0; i < kRunTicks; ++i) {
      // What the monitor is about to be shown: the readings taken at the end of the last tick.
      const stapes::hardware::SafetySample safety = sim.arm().read_safety_sensors();
      const stapes::hardware::SensorSample control = sim.arm().read_sensors();
      sim.step();
      const Eigen::Vector3d tracked_tip = safety.tracked_tool_pose.translation();
      const Eigen::Vector3d tip_from_links =
          stapes::kinematics::forward_kinematics(scenario.safety.arm, safety.link_positions_rad)
              .translation();
      tracker_against_links =
          std::max(tracker_against_links, (tracked_tip - tip_from_links).norm());
      tracked_tip_off_line =
          std::max(tracked_tip_off_line, scenario.trajectory.radial_distance(tracked_tip));
      tracked_shaft_at_entry = std::max(
          tracked_shaft_at_entry,
          stapes::task::axis_distance(safety.tracked_tool_pose, scenario.trajectory.entry()));
      motor_against_link =
          std::max(motor_against_link,
                   (control.joint_positions_rad - safety.link_positions_rad).cwiseAbs().maxCoeff());
      link_against_target = std::max(
          link_against_target, (safety.link_positions_rad - sim.target()).cwiseAbs().maxCoeff());
      tool_force = std::max(tool_force, safety.tool_wrench.head<3>().norm());
    }
    ASSERT_EQ(sim.monitor().cause(), stapes::safety::StopCause::none);
  }
  RecordProperty("margin_tracker_against_links_um",
                 std::to_string(tracker_against_links * kMicrometresPerMetre));
  RecordProperty("margin_tracked_tip_off_line_um",
                 std::to_string(tracked_tip_off_line * kMicrometresPerMetre));
  RecordProperty("margin_tracked_shaft_at_entry_um",
                 std::to_string(tracked_shaft_at_entry * kMicrometresPerMetre));
  RecordProperty("margin_motor_against_link_mrad", std::to_string(motor_against_link * 1e3));
  RecordProperty("margin_link_against_target_mrad", std::to_string(link_against_target * 1e3));
  RecordProperty("margin_tool_force_n", std::to_string(tool_force));

  const stapes::safety::SafetyConfig limits = KeyholeScenario::reference(1).safety;
  EXPECT_LT(tracker_against_links, limits.max_tracker_disagreement_m);
  EXPECT_LT(tracked_tip_off_line, limits.corridor.radius_m);
  EXPECT_LT(tracked_shaft_at_entry, limits.corridor.radius_m);
  EXPECT_LT(motor_against_link, limits.max_encoder_disagreement_rad.minCoeff());
  EXPECT_LT(link_against_target, limits.max_tracking_error_rad.minCoeff());
  EXPECT_LT(tool_force, limits.max_tool_force_n);
}

TEST(KeyholeDrilling, EveryJointMovesAlongTheReferenceTunnel) {
  // A joint that does not move could hide a fault, so the reference scenario moves them all.
  stapes::test::verifies("SRS-KEY-006");
  KeyholeSimulation sim(KeyholeScenario::reference(1));
  const JointVector start = sim.arm().state().q;
  for (int i = 0; i < kRunTicks; ++i) {
    sim.step();
  }
  const JointVector travel = (sim.arm().state().q - start).cwiseAbs();
  RecordProperty("smallest_joint_travel_mrad", std::to_string(travel.minCoeff() * 1e3));
  EXPECT_GT(travel.minCoeff(), 2e-3);
}

TEST(KeyholeDrilling, HoldsTheLineUnderARampedThrustOfTwoNewtons) {
  stapes::test::verifies("SRS-KEY-007");
  constexpr double kThrust = 2.0;
  double worst = 0.0;
  for (int seed = 1; seed <= 3; ++seed) {
    const KeyholeScenario scenario = KeyholeScenario::reference(static_cast<std::uint64_t>(seed));
    const stapes::test::RunResult result = stapes::test::run_with_thrust(scenario, kThrust);
    EXPECT_EQ(result.cause, stapes::safety::StopCause::none);
    worst = std::max(worst, result.max_radial_m);
  }
  RecordProperty("thrust_2n_max_radial_um", std::to_string(worst * kMicrometresPerMetre));
  EXPECT_LT(worst, 0.2e-3);
}

TEST(KeyholeSimulation, RefusesAPlanItCannotReachBeforeMoving) {
  stapes::test::verifies("SRS-KEY-005");
  KeyholeScenario scenario = KeyholeScenario::reference(1);
  scenario.trajectory = stapes::task::KeyholeTrajectory({5.0, 0.0, 0.0}, {5.1, 0.0, 0.0},
                                                        stapes::math::Pose::Identity());
  EXPECT_THROW(KeyholeSimulation{scenario}, std::invalid_argument);
}

TEST(KeyholeSimulation, StartsOnThePlan) {
  stapes::test::verifies("SRS-KEY-005");
  const KeyholeScenario scenario = KeyholeScenario::reference(1);
  const KeyholeSimulation sim(scenario);
  const stapes::math::Pose tool = sim.true_tool_pose();
  EXPECT_LT(scenario.trajectory.radial_distance(tool.translation()), 2e-6);
  EXPECT_NEAR(scenario.trajectory.depth_of(tool.translation()), 0.0, 2e-6);
  EXPECT_LT(stapes::task::axis_distance(tool, scenario.trajectory.entry()), 2e-6);
  EXPECT_FALSE(sim.stop_tick().has_value());
  EXPECT_NEAR(sim.feed_duration_s(), 2.2, 1e-12);
}

TEST(RemoteCentreOfMotion, TheToolPivotsAboutAFixedPointUnderClosedLoopControl) {
  // The secondary scenario: the same constraint, with the tip moving sideways as in laparoscopy
  // instead of along the axis. The tip moves at about 2 mm/s; the offset grows with speed, and
  // ADR 0009 records by how much.
  stapes::test::verifies("SRS-KEY-008");
  const stapes::plant::SimulatedArmConfig config = stapes::plant::SimulatedArmConfig::reference(21);
  stapes::plant::SimulatedArm arm(config);
  stapes::control::JointController controller(config.model, stapes::control::reference_gains(),
                                              stapes::control::reference_gearbox_estimates(), 1e-3);
  const stapes::kinematics::ArmModel& model = config.model.arm();
  const stapes::math::Pose start = stapes::kinematics::forward_kinematics(model, config.initial_q);
  const Eigen::Vector3d axis = stapes::task::tool_axis(start);
  const Eigen::Vector3d fulcrum = start.translation() - (40e-3 * axis);
  const Eigen::Vector3d side_a = axis.unitOrthogonal();
  const Eigen::Vector3d side_b = axis.cross(side_a);

  constexpr double kCircleRadius = 5e-3;
  constexpr double kPeriodS = 16.0;
  constexpr int kRampTicks = 1000;
  constexpr int kTicks = 17000;
  JointVector target = config.initial_q;
  double worst_offset = 0.0;
  double largest_tilt = 0.0;
  for (int tick = 0; tick < kTicks; ++tick) {
    const double time = 1e-3 * tick;
    const double ramp = std::min(1.0, static_cast<double>(tick) / kRampTicks);
    const double angle = 2.0 * std::numbers::pi * time / kPeriodS;
    const Eigen::Vector3d tip =
        start.translation() +
        (ramp * kCircleRadius * ((std::cos(angle) * side_a) + (std::sin(angle) * side_b) - side_a));
    const stapes::kinematics::IkResult solved = stapes::kinematics::solve_ik(
        model, stapes::task::pose_through_fulcrum(tip, fulcrum, start), target);
    ASSERT_TRUE(solved.converged()) << "tick " << tick;
    target = solved.q;
    stapes::test::control_step(arm, controller, target);
    arm.tick();

    const stapes::math::Pose tool = stapes::kinematics::forward_kinematics(model, arm.state().q);
    worst_offset = std::max(worst_offset, stapes::task::axis_distance(tool, fulcrum));
    largest_tilt =
        std::max(largest_tilt, std::acos(std::min(1.0, stapes::task::tool_axis(tool).dot(axis))));
  }
  RecordProperty("rcm_max_axis_offset_um", std::to_string(worst_offset * kMicrometresPerMetre));
  RecordProperty("rcm_largest_tilt_mrad", std::to_string(largest_tilt * 1e3));
  EXPECT_LT(worst_offset, 0.2e-3);
  EXPECT_GT(largest_tilt, 0.1);  // It really pivoted.
}

}  // namespace
