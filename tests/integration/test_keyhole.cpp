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
