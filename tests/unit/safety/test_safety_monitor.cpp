// The specification of SafetyMonitor, written before its implementation. Each test isolates one
// behaviour from the header's contract.

#include <array>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/safety/safety_monitor.hpp"

namespace {

using stapes::kinematics::ArmModel;
using stapes::kinematics::forward_kinematics;
using stapes::kinematics::JointVector;
using stapes::kinematics::reference_home;
using stapes::math::Wrench;
using stapes::safety::Corridor;
using stapes::safety::reference_safety_config;
using stapes::safety::SafetyConfig;
using stapes::safety::SafetyInput;
using stapes::safety::SafetyMonitor;
using stapes::safety::StopCause;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTunnelLength = 10e-3;
constexpr double kRadius = 0.3e-3;
constexpr int kDebounce = 3;

SafetyConfig make_config() {
  const stapes::math::Pose start = forward_kinematics(ArmModel::reference(), reference_home());
  const Eigen::Vector3d entry = start.translation();
  const Eigen::Vector3d target = entry + (kTunnelLength * start.linear().col(0));
  SafetyConfig config = reference_safety_config(Corridor{entry, target, kRadius, 0.2e-3});
  config.debounce_checks = kDebounce;
  return config;
}

// A consistent reading: the arm is at home with the tip at the tunnel's entry, on the axis.
SafetyInput clean_input(std::int64_t now = 0) {
  const JointVector q = reference_home();
  SafetyInput input{};
  input.safety.link_positions_rad = q;
  input.safety.tracked_tool_pose = forward_kinematics(ArmModel::reference(), q);
  input.safety.tool_wrench = Wrench::Zero();
  input.safety.stamp_ticks = now;
  input.control.joint_positions_rad = q;
  input.control.tool_wrench = Wrench::Zero();
  input.control.stamp_ticks = now;
  input.target = q;
  input.now_ticks = now;
  return input;
}

Eigen::Vector3d tunnel_direction(const SafetyConfig& config) {
  return (config.corridor.target_m - config.corridor.entry_m).normalized();
}

// Any direction perpendicular to the tunnel.
Eigen::Vector3d sideways(const SafetyConfig& config) {
  return tunnel_direction(config).unitOrthogonal();
}

// Calls `check` with `input` until the debounce has elapsed, and returns the final answer.
StopCause check_repeatedly(SafetyMonitor& monitor, const SafetyInput& input, int times) {
  StopCause cause = StopCause::none;
  for (int i = 0; i < times; ++i) {
    SafetyInput current = input;
    current.now_ticks = input.now_ticks + i;
    current.safety.stamp_ticks = current.now_ticks;
    current.control.stamp_ticks = current.now_ticks;
    cause = monitor.check(current);
  }
  return cause;
}

// ---- Configuration ---------------------------------------------------------------------------

TEST(SafetyMonitorConfig, TheReferenceConfigurationIsAccepted) {
  stapes::test::verifies("SRS-SAF-001");
  EXPECT_NO_THROW(SafetyMonitor{make_config()});
}

TEST(SafetyMonitorConfig, ZeroMarginsAreAllowed) {
  stapes::test::verifies("SRS-SAF-001");
  SafetyConfig config = make_config();
  config.joint_limit_margin_rad = 0.0;
  config.corridor.overshoot_margin_m = 0.0;
  EXPECT_NO_THROW(SafetyMonitor{config});
}

TEST(SafetyMonitorConfig, InvalidValuesAreRejected) {
  stapes::test::verifies("SRS-SAF-001");
  using Change = std::function<void(SafetyConfig&)>;
  const std::array<std::pair<const char*, Change>, 26> cases{{
      {"entry NaN", [](SafetyConfig& c) { c.corridor.entry_m(0) = kNaN; }},
      {"target infinite", [](SafetyConfig& c) { c.corridor.target_m(1) = kInf; }},
      {"tunnel with no length", [](SafetyConfig& c) { c.corridor.target_m = c.corridor.entry_m; }},
      {"radius zero", [](SafetyConfig& c) { c.corridor.radius_m = 0.0; }},
      {"radius negative", [](SafetyConfig& c) { c.corridor.radius_m = -1.0; }},
      {"radius NaN", [](SafetyConfig& c) { c.corridor.radius_m = kNaN; }},
      {"overshoot negative", [](SafetyConfig& c) { c.corridor.overshoot_margin_m = -1.0; }},
      {"overshoot NaN", [](SafetyConfig& c) { c.corridor.overshoot_margin_m = kNaN; }},
      {"margin negative", [](SafetyConfig& c) { c.joint_limit_margin_rad = -0.1; }},
      {"margin NaN", [](SafetyConfig& c) { c.joint_limit_margin_rad = kNaN; }},
      {"period zero", [](SafetyConfig& c) { c.period_s = 0.0; }},
      {"period NaN", [](SafetyConfig& c) { c.period_s = kNaN; }},
      {"velocity zero", [](SafetyConfig& c) { c.max_velocity_rad_s(2) = 0.0; }},
      {"velocity NaN", [](SafetyConfig& c) { c.max_velocity_rad_s(3) = kNaN; }},
      {"force zero", [](SafetyConfig& c) { c.max_tool_force_n = 0.0; }},
      {"force infinite", [](SafetyConfig& c) { c.max_tool_force_n = kInf; }},
      {"tracker limit zero", [](SafetyConfig& c) { c.max_tracker_disagreement_m = 0.0; }},
      {"tracker limit NaN", [](SafetyConfig& c) { c.max_tracker_disagreement_m = kNaN; }},
      {"encoder limit zero", [](SafetyConfig& c) { c.max_encoder_disagreement_rad(0) = 0.0; }},
      {"encoder limit NaN", [](SafetyConfig& c) { c.max_encoder_disagreement_rad(5) = kNaN; }},
      {"tracking limit negative", [](SafetyConfig& c) { c.max_tracking_error_rad(1) = -1.0; }},
      {"tracking limit NaN", [](SafetyConfig& c) { c.max_tracking_error_rad(4) = kNaN; }},
      {"sample age zero", [](SafetyConfig& c) { c.max_sample_age_ticks = 0; }},
      {"sample age negative", [](SafetyConfig& c) { c.max_sample_age_ticks = -1; }},
      {"debounce zero", [](SafetyConfig& c) { c.debounce_checks = 0; }},
      {"debounce negative", [](SafetyConfig& c) { c.debounce_checks = -1; }},
  }};
  for (const auto& [name, change] : cases) {
    SCOPED_TRACE(name);
    SafetyConfig config = make_config();
    change(config);
    EXPECT_THROW(SafetyMonitor{config}, std::invalid_argument);
  }
}

// ---- A healthy robot -------------------------------------------------------------------------

TEST(SafetyMonitor, StartsClear) {
  stapes::test::verifies("SRS-SAF-002");
  const SafetyMonitor monitor(make_config());
  EXPECT_FALSE(monitor.stopped());
  EXPECT_EQ(monitor.cause(), StopCause::none);
}

TEST(SafetyMonitor, ACleanInputNeverStops) {
  stapes::test::verifies("SRS-SAF-002");
  SafetyMonitor monitor(make_config());
  EXPECT_EQ(check_repeatedly(monitor, clean_input(), 1000), StopCause::none);
  EXPECT_FALSE(monitor.stopped());
}

// ---- Non-finite input ------------------------------------------------------------------------

TEST(SafetyMonitor, NonFiniteValuesInAnyFieldStopAtOnce) {
  stapes::test::verifies("SRS-SAF-003");
  using Change = std::function<void(SafetyInput&, double)>;
  const std::array<std::pair<const char*, Change>, 6> fields{{
      {"link position", [](SafetyInput& i, double v) { i.safety.link_positions_rad(2) = v; }},
      {"tracked pose", [](SafetyInput& i, double v) { i.safety.tracked_tool_pose(1, 3) = v; }},
      {"safety wrench", [](SafetyInput& i, double v) { i.safety.tool_wrench(4) = v; }},
      {"motor position", [](SafetyInput& i, double v) { i.control.joint_positions_rad(5) = v; }},
      {"control wrench", [](SafetyInput& i, double v) { i.control.tool_wrench(0) = v; }},
      {"target", [](SafetyInput& i, double v) { i.target(3) = v; }},
  }};
  for (const auto& [name, change] : fields) {
    for (const double bad : {kNaN, kInf, -kInf}) {
      SCOPED_TRACE(name);
      SafetyMonitor monitor(make_config());
      SafetyInput input = clean_input();
      change(input, bad);
      EXPECT_EQ(monitor.check(input), StopCause::non_finite_input);
      EXPECT_TRUE(monitor.stopped());
    }
  }
}

// ---- Stale data ------------------------------------------------------------------------------

TEST(SafetyMonitor, ASampleAtTheMaximumAgeIsAccepted) {
  stapes::test::verifies("SRS-SAF-004");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input(100);
  input.safety.stamp_ticks = 100 - config.max_sample_age_ticks;
  input.control.stamp_ticks = 100 - config.max_sample_age_ticks;
  EXPECT_EQ(monitor.check(input), StopCause::none);
}

TEST(SafetyMonitor, AnOldSampleOnEitherChannelStopsAtOnce) {
  stapes::test::verifies("SRS-SAF-004");
  const SafetyConfig config = make_config();
  {
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input(100);
    input.safety.stamp_ticks = 100 - config.max_sample_age_ticks - 1;
    EXPECT_EQ(monitor.check(input), StopCause::stale_data);
  }
  {
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input(100);
    input.control.stamp_ticks = 100 - config.max_sample_age_ticks - 1;
    EXPECT_EQ(monitor.check(input), StopCause::stale_data);
  }
}

TEST(SafetyMonitor, ASampleFromTheFutureIsNotTrusted) {
  stapes::test::verifies("SRS-SAF-004");
  {
    SafetyMonitor monitor(make_config());
    SafetyInput input = clean_input(100);
    input.safety.stamp_ticks = 101;
    EXPECT_EQ(monitor.check(input), StopCause::stale_data);
  }
  {
    SafetyMonitor monitor(make_config());
    SafetyInput input = clean_input(100);
    input.control.stamp_ticks = 101;
    EXPECT_EQ(monitor.check(input), StopCause::stale_data);
  }
}

// ---- Joint limits ----------------------------------------------------------------------------

TEST(SafetyMonitor, EveryJointIsCheckedAgainstBothLimitsWithItsMargin) {
  stapes::test::verifies("SRS-SAF-005");
  const SafetyConfig config = make_config();
  const double margin = config.joint_limit_margin_rad;
  constexpr double kJustOver = 1e-9;
  Eigen::Index row = 0;
  for (const stapes::kinematics::RevoluteJoint& joint : config.arm.joints()) {
    SCOPED_TRACE("joint " + std::to_string(row));
    const std::array<std::pair<double, StopCause>, 4> cases{{
        {joint.lower_rad + margin + kJustOver, StopCause::none},
        {joint.upper_rad - margin - kJustOver, StopCause::none},
        {joint.lower_rad + margin - kJustOver, StopCause::joint_limit},
        {joint.upper_rad - margin + kJustOver, StopCause::joint_limit},
    }};
    for (const auto& [angle, expected] : cases) {
      SafetyMonitor monitor(config);
      SafetyInput input = clean_input();
      input.safety.link_positions_rad(row) = angle;
      EXPECT_EQ(monitor.check(input), expected);
    }
    ++row;
  }
}

TEST(SafetyMonitor, TheJointLimitUsesTheLinkEncoderNotTheMotorEncoder) {
  stapes::test::verifies("SRS-SAF-005");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.control.joint_positions_rad(0) = config.arm.joints()[0].upper_rad + 1.0;
  EXPECT_EQ(monitor.check(input), StopCause::none);
}

// ---- Joint velocity --------------------------------------------------------------------------

TEST(SafetyMonitor, TheFirstCheckHasNoVelocityToJudge) {
  stapes::test::verifies("SRS-SAF-006");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.safety.link_positions_rad(1) += 0.1;  // Far from the previous reading, if there were one.
  EXPECT_EQ(monitor.check(input), StopCause::none);
}

TEST(SafetyMonitor, EveryJointIsCheckedForSpeedInBothDirections) {
  stapes::test::verifies("SRS-SAF-006");
  SafetyConfig config = make_config();
  // Different limits per joint, so that using the wrong joint's limit fails.
  for (Eigen::Index row = 0; row < config.max_velocity_rad_s.size(); ++row) {
    config.max_velocity_rad_s(row) = 0.1 + (0.05 * static_cast<double>(row));
  }
  for (Eigen::Index row = 0; row < config.max_velocity_rad_s.size(); ++row) {
    SCOPED_TRACE("joint " + std::to_string(row));
    const double limit_step = config.max_velocity_rad_s(row) * config.period_s;
    for (const double sign : {1.0, -1.0}) {
      const std::array<std::pair<double, StopCause>, 2> cases{{
          {0.99 * limit_step, StopCause::none},
          {1.01 * limit_step, StopCause::joint_velocity},
      }};
      for (const auto& [step, expected] : cases) {
        SafetyMonitor monitor(config);
        ASSERT_EQ(monitor.check(clean_input(0)), StopCause::none);
        SafetyInput moved = clean_input(1);
        moved.safety.link_positions_rad(row) += sign * step;
        EXPECT_EQ(monitor.check(moved), expected);
      }
    }
  }
}

TEST(SafetyMonitor, VelocityComparesConsecutiveReadingsOnly) {
  // A slow move covers a long way in total. It would trip a check that kept comparing against
  // the first reading.
  stapes::test::verifies("SRS-SAF-006");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  // The tool roll joint turns the tool about its own axis, so the tip stays on the line and
  // nothing but velocity could object.
  constexpr Eigen::Index kRoll = 5;
  const double step = 0.5 * config.max_velocity_rad_s(kRoll) * config.period_s;
  for (int i = 0; i < 100; ++i) {
    SafetyInput input = clean_input(i);
    input.safety.link_positions_rad(kRoll) += step * i;
    input.control.joint_positions_rad(kRoll) += step * i;
    input.target(kRoll) += step * i;
    input.safety.tracked_tool_pose =
        forward_kinematics(config.arm, input.safety.link_positions_rad);
    ASSERT_EQ(monitor.check(input), StopCause::none) << "step " << i;
  }
}

// ---- Tool force ------------------------------------------------------------------------------

TEST(SafetyMonitor, ForceStopsOnlyAfterTheDebounce) {
  stapes::test::verifies("SRS-SAF-007");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.safety.tool_wrench(0) = 1.01 * config.max_tool_force_n;
  EXPECT_EQ(check_repeatedly(monitor, input, kDebounce - 1), StopCause::none);
  input.now_ticks = kDebounce - 1;
  input.safety.stamp_ticks = input.now_ticks;
  input.control.stamp_ticks = input.now_ticks;
  EXPECT_EQ(monitor.check(input), StopCause::tool_force);
}

TEST(SafetyMonitor, ACleanCheckRestartsTheDebounce) {
  stapes::test::verifies("SRS-SAF-007");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput over = clean_input();
  over.safety.tool_wrench(1) = 2.0 * config.max_tool_force_n;
  for (int round = 0; round < 10; ++round) {
    EXPECT_EQ(check_repeatedly(monitor, over, kDebounce - 1), StopCause::none);
    EXPECT_EQ(check_repeatedly(monitor, clean_input(), 1), StopCause::none);
  }
}

TEST(SafetyMonitor, ForceIsJudgedByMagnitudeAcrossAllThreeAxes) {
  stapes::test::verifies("SRS-SAF-007");
  const SafetyConfig config = make_config();
  // Each axis is below the limit, but together they exceed it.
  const double per_axis = 0.7 * config.max_tool_force_n;
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.safety.tool_wrench.head<3>().setConstant(-per_axis);
  EXPECT_EQ(check_repeatedly(monitor, input, kDebounce), StopCause::tool_force);
}

TEST(SafetyMonitor, ForceBelowTheLimitAndTorqueDoNotCount) {
  stapes::test::verifies("SRS-SAF-007");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.safety.tool_wrench(2) = 0.99 * config.max_tool_force_n;
  input.safety.tool_wrench.tail<3>().setConstant(10.0 * config.max_tool_force_n);
  EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
}

// ---- Corridor --------------------------------------------------------------------------------

// The corridor tests move only the tracker, so the disagreement check is widened out of the way.
SafetyConfig corridor_config() {
  SafetyConfig config = make_config();
  config.max_tracker_disagreement_m = 1.0;
  return config;
}

TEST(SafetyMonitor, TheTipMustStayWithinTheRadius) {
  stapes::test::verifies("SRS-SAF-008");
  const SafetyConfig config = corridor_config();
  const Eigen::Vector3d side = sideways(config);
  const Eigen::Vector3d along = tunnel_direction(config);
  const stapes::math::Pose on_axis = clean_input().safety.tracked_tool_pose;
  {
    // The tip at mid depth, off the line. The tool stays parallel to the tunnel.
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input();
    input.safety.tracked_tool_pose.translation() =
        on_axis.translation() + (5e-3 * along) + (0.99 * kRadius * side);
    EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
  }
  {
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input();
    input.safety.tracked_tool_pose.translation() =
        on_axis.translation() + (5e-3 * along) + (1.01 * kRadius * side);
    EXPECT_EQ(check_repeatedly(monitor, input, kDebounce), StopCause::corridor);
  }
}

TEST(SafetyMonitor, TheCorridorStopWaitsForTheDebounce) {
  stapes::test::verifies("SRS-SAF-008");
  const SafetyConfig config = corridor_config();
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.safety.tracked_tool_pose.translation() += 2.0 * kRadius * sideways(config);
  EXPECT_EQ(check_repeatedly(monitor, input, kDebounce - 1), StopCause::none);
  EXPECT_EQ(check_repeatedly(monitor, clean_input(10), 1), StopCause::none);
  EXPECT_EQ(check_repeatedly(monitor, input, kDebounce - 1), StopCause::none);
}

TEST(SafetyMonitor, TheToolAxisMustPassThroughTheEntry) {
  // The tip is on the line, deep in the tunnel, but the tool is tilted. Its shaft would be
  // outside the keyhole at the entry.
  stapes::test::verifies("SRS-SAF-008");
  const SafetyConfig config = corridor_config();
  const Eigen::Vector3d along = tunnel_direction(config);
  const stapes::math::Pose start = clean_input().safety.tracked_tool_pose;
  constexpr double kDepth = 10e-3;
  const Eigen::Vector3d turn_axis = sideways(config);

  const auto tilted_by = [&](double offset_at_entry) {
    SafetyInput input = clean_input();
    const double tilt = offset_at_entry / kDepth;
    input.safety.tracked_tool_pose.linear() =
        Eigen::AngleAxisd(tilt, turn_axis).toRotationMatrix() * start.linear();
    input.safety.tracked_tool_pose.translation() = start.translation() + (kDepth * along);
    return input;
  };
  {
    SafetyMonitor monitor(config);
    EXPECT_EQ(check_repeatedly(monitor, tilted_by(0.9 * kRadius), 10 * kDebounce), StopCause::none);
  }
  {
    SafetyMonitor monitor(config);
    EXPECT_EQ(check_repeatedly(monitor, tilted_by(1.1 * kRadius), kDebounce), StopCause::corridor);
  }
}

TEST(SafetyMonitor, TheTipMustNotPassTheTargetBeyondTheMargin) {
  stapes::test::verifies("SRS-SAF-008");
  const SafetyConfig config = corridor_config();
  const Eigen::Vector3d along = tunnel_direction(config);
  const double margin = config.corridor.overshoot_margin_m;
  const stapes::math::Pose start = clean_input().safety.tracked_tool_pose;
  {
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input();
    input.safety.tracked_tool_pose.translation() =
        start.translation() + ((kTunnelLength + (0.99 * margin)) * along);
    EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
  }
  {
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input();
    input.safety.tracked_tool_pose.translation() =
        start.translation() + ((kTunnelLength + (1.01 * margin)) * along);
    EXPECT_EQ(check_repeatedly(monitor, input, kDebounce), StopCause::corridor);
  }
}

TEST(SafetyMonitor, BackingOutAlongTheAxisIsAllowed) {
  // Retreating is the safe direction. Only the far end of the tunnel has a depth limit.
  stapes::test::verifies("SRS-SAF-008");
  const SafetyConfig config = corridor_config();
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.safety.tracked_tool_pose.translation() -= 50e-3 * tunnel_direction(config);
  EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
}

// ---- Tracker against link encoders -----------------------------------------------------------

TEST(SafetyMonitor, TheTrackerMustAgreeWithTheLinkEncodersAboutTheTip) {
  stapes::test::verifies("SRS-SAF-009");
  const SafetyConfig config = make_config();
  const Eigen::Vector3d along = tunnel_direction(config);
  const double limit = config.max_tracker_disagreement_m;
  {
    // Moving along the tunnel stays inside the corridor, so only this check can object.
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input();
    input.safety.tracked_tool_pose.translation() += 0.99 * limit * along;
    EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
  }
  {
    SafetyMonitor monitor(config);
    SafetyInput input = clean_input();
    input.safety.tracked_tool_pose.translation() += 1.01 * limit * along;
    EXPECT_EQ(check_repeatedly(monitor, input, kDebounce - 1), StopCause::none);
    input.now_ticks = kDebounce - 1;
    input.safety.stamp_ticks = input.now_ticks;
    input.control.stamp_ticks = input.now_ticks;
    EXPECT_EQ(monitor.check(input), StopCause::tracker_disagreement);
  }
}

TEST(SafetyMonitor, TheTrackerComparisonUsesTheLinkEncodersNotTheMotorEncoders) {
  stapes::test::verifies("SRS-SAF-009");
  SafetyConfig config = make_config();
  config.max_encoder_disagreement_rad.setConstant(10.0);
  SafetyMonitor monitor(config);
  SafetyInput input = clean_input();
  input.control.joint_positions_rad(1) += 0.5;  // Would move the tip a long way, if it counted.
  EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
}

// ---- Motor encoders against link encoders ----------------------------------------------------

TEST(SafetyMonitor, EachMotorEncoderMustAgreeWithItsLinkEncoder) {
  stapes::test::verifies("SRS-SAF-010");
  SafetyConfig config = make_config();
  // Different limits per joint, so that using the wrong joint's limit fails.
  for (Eigen::Index row = 0; row < config.max_encoder_disagreement_rad.size(); ++row) {
    config.max_encoder_disagreement_rad(row) = 1e-3 + (2e-4 * static_cast<double>(row));
  }
  for (Eigen::Index row = 0; row < config.max_encoder_disagreement_rad.size(); ++row) {
    SCOPED_TRACE("joint " + std::to_string(row));
    const double limit = config.max_encoder_disagreement_rad(row);
    for (const double sign : {1.0, -1.0}) {
      {
        SafetyMonitor monitor(config);
        SafetyInput input = clean_input();
        input.control.joint_positions_rad(row) += sign * 0.99 * limit;
        EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
      }
      {
        SafetyMonitor monitor(config);
        SafetyInput input = clean_input();
        input.control.joint_positions_rad(row) += sign * 1.01 * limit;
        EXPECT_EQ(check_repeatedly(monitor, input, kDebounce - 1), StopCause::none);
        input.now_ticks = kDebounce - 1;
        input.safety.stamp_ticks = input.now_ticks;
        input.control.stamp_ticks = input.now_ticks;
        EXPECT_EQ(monitor.check(input), StopCause::encoder_disagreement);
      }
    }
  }
}

// ---- Tracking error --------------------------------------------------------------------------

TEST(SafetyMonitor, EachLinkMustStayNearItsTarget) {
  stapes::test::verifies("SRS-SAF-011");
  SafetyConfig config = make_config();
  for (Eigen::Index row = 0; row < config.max_tracking_error_rad.size(); ++row) {
    config.max_tracking_error_rad(row) = 1e-3 + (2e-4 * static_cast<double>(row));
  }
  for (Eigen::Index row = 0; row < config.max_tracking_error_rad.size(); ++row) {
    SCOPED_TRACE("joint " + std::to_string(row));
    const double limit = config.max_tracking_error_rad(row);
    for (const double sign : {1.0, -1.0}) {
      {
        SafetyMonitor monitor(config);
        SafetyInput input = clean_input();
        input.target(row) += sign * 0.99 * limit;
        EXPECT_EQ(check_repeatedly(monitor, input, 10 * kDebounce), StopCause::none);
      }
      {
        SafetyMonitor monitor(config);
        SafetyInput input = clean_input();
        input.target(row) += sign * 1.01 * limit;
        EXPECT_EQ(check_repeatedly(monitor, input, kDebounce - 1), StopCause::none);
        input.now_ticks = kDebounce - 1;
        input.safety.stamp_ticks = input.now_ticks;
        input.control.stamp_ticks = input.now_ticks;
        EXPECT_EQ(monitor.check(input), StopCause::tracking_error);
      }
    }
  }
}

// ---- Debounce counts are per check -----------------------------------------------------------

TEST(SafetyMonitor, EachCheckKeepsItsOwnDebounceCount) {
  // Two different conditions in turn are not one condition lasting twice as long.
  stapes::test::verifies("SRS-SAF-012");
  const SafetyConfig config = corridor_config();
  SafetyInput force = clean_input();
  force.safety.tool_wrench(0) = 2.0 * config.max_tool_force_n;
  SafetyInput off_line = clean_input();
  off_line.safety.tracked_tool_pose.translation() += 2.0 * kRadius * sideways(config);

  SafetyMonitor monitor(config);
  for (int round = 0; round < 5; ++round) {
    EXPECT_EQ(check_repeatedly(monitor, force, 1), StopCause::none);
    EXPECT_EQ(check_repeatedly(monitor, off_line, 1), StopCause::none);
  }
}

TEST(SafetyMonitor, AConditionKeepsCountingWhileAnotherOneComesAndGoes) {
  stapes::test::verifies("SRS-SAF-012");
  const SafetyConfig config = corridor_config();
  SafetyInput steady = clean_input();
  steady.target(0) += 2.0 * config.max_tracking_error_rad(0);
  SafetyInput both = steady;
  both.safety.tool_wrench(0) = 2.0 * config.max_tool_force_n;

  SafetyMonitor monitor(config);
  EXPECT_EQ(check_repeatedly(monitor, steady, 1), StopCause::none);
  EXPECT_EQ(check_repeatedly(monitor, both, 1), StopCause::none);
  EXPECT_EQ(check_repeatedly(monitor, steady, 1), StopCause::tracking_error);
}

// ---- Latching --------------------------------------------------------------------------------

TEST(SafetyMonitor, AStopIsLatchedThroughCleanInput) {
  stapes::test::verifies("SRS-SAF-013");
  SafetyMonitor monitor(make_config());
  SafetyInput bad = clean_input();
  bad.target(0) = kNaN;
  ASSERT_EQ(monitor.check(bad), StopCause::non_finite_input);
  EXPECT_EQ(check_repeatedly(monitor, clean_input(), 100), StopCause::non_finite_input);
  EXPECT_TRUE(monitor.stopped());
  EXPECT_EQ(monitor.cause(), StopCause::non_finite_input);
}

TEST(SafetyMonitor, TheFirstCauseIsKept) {
  stapes::test::verifies("SRS-SAF-013");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput first = clean_input();
  first.safety.stamp_ticks = -100;
  ASSERT_EQ(monitor.check(first), StopCause::stale_data);
  SafetyInput second = clean_input();
  second.target(0) = kNaN;
  EXPECT_EQ(monitor.check(second), StopCause::stale_data);
}

// ---- Reset -----------------------------------------------------------------------------------

TEST(SafetyMonitorReset, DoesNothingWhileNotStopped) {
  stapes::test::verifies("SRS-SAF-014");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput over = clean_input();
  over.safety.tool_wrench(0) = 2.0 * config.max_tool_force_n;
  EXPECT_EQ(check_repeatedly(monitor, over, kDebounce - 1), StopCause::none);

  // Even with an input that would fail: there is nothing to clear, and the count must survive.
  EXPECT_TRUE(monitor.try_reset(over));
  EXPECT_FALSE(monitor.stopped());
  over.now_ticks = kDebounce - 1;
  over.safety.stamp_ticks = over.now_ticks;
  over.control.stamp_ticks = over.now_ticks;
  EXPECT_EQ(monitor.check(over), StopCause::tool_force);
}

TEST(SafetyMonitorReset, IsRefusedWhileAnyConditionPersists) {
  stapes::test::verifies("SRS-SAF-014");
  const SafetyConfig config = make_config();
  using Change = std::function<void(SafetyInput&)>;
  const std::array<std::pair<const char*, Change>, 8> conditions{{
      {"not finite", [](SafetyInput& i) { i.target(0) = kNaN; }},
      {"stale", [&](SafetyInput& i) { i.safety.stamp_ticks -= config.max_sample_age_ticks + 1; }},
      {"joint limit",
       [&](SafetyInput& i) {
         i.safety.link_positions_rad(0) = config.arm.joints()[0].upper_rad + 0.1;
       }},
      {"force", [&](SafetyInput& i) { i.safety.tool_wrench(0) = 2.0 * config.max_tool_force_n; }},
      {"corridor",
       [&](SafetyInput& i) {
         i.safety.tracked_tool_pose.translation() += 2.0 * kRadius * sideways(config);
       }},
      {"tracker",
       [&](SafetyInput& i) {
         i.safety.tracked_tool_pose.translation() +=
             2.0 * config.max_tracker_disagreement_m * tunnel_direction(config);
       }},
      {"encoders",
       [&](SafetyInput& i) {
         i.control.joint_positions_rad(1) += 2.0 * config.max_encoder_disagreement_rad(1);
       }},
      {"tracking", [&](SafetyInput& i) { i.target(2) += 2.0 * config.max_tracking_error_rad(2); }},
  }};
  for (const auto& [name, change] : conditions) {
    SCOPED_TRACE(name);
    SafetyMonitor monitor(config);
    SafetyInput bad = clean_input(100);
    bad.safety.stamp_ticks = -100;  // Stops the monitor, with a known cause.
    ASSERT_EQ(monitor.check(bad), StopCause::stale_data);

    SafetyInput still_bad = clean_input(100);
    change(still_bad);
    EXPECT_FALSE(monitor.try_reset(still_bad));
    EXPECT_TRUE(monitor.stopped());
    EXPECT_EQ(monitor.cause(), StopCause::stale_data);
  }
}

TEST(SafetyMonitorReset, SucceedsOnceTheConditionHasCleared) {
  stapes::test::verifies("SRS-SAF-014");
  SafetyMonitor monitor(make_config());
  SafetyInput bad = clean_input();
  bad.target(0) = kNaN;
  ASSERT_EQ(monitor.check(bad), StopCause::non_finite_input);

  EXPECT_TRUE(monitor.try_reset(clean_input()));
  EXPECT_FALSE(monitor.stopped());
  EXPECT_EQ(monitor.cause(), StopCause::none);
  EXPECT_EQ(monitor.check(clean_input(1)), StopCause::none);
}

TEST(SafetyMonitorReset, ForgetsDebounceCounts) {
  stapes::test::verifies("SRS-SAF-014");
  const SafetyConfig config = make_config();
  SafetyMonitor monitor(config);
  SafetyInput over = clean_input();
  over.safety.tool_wrench(0) = 2.0 * config.max_tool_force_n;
  ASSERT_EQ(check_repeatedly(monitor, over, kDebounce), StopCause::tool_force);
  ASSERT_TRUE(monitor.try_reset(clean_input(10)));

  EXPECT_EQ(check_repeatedly(monitor, over, kDebounce - 1), StopCause::none);
  EXPECT_FALSE(monitor.stopped());
}

TEST(SafetyMonitorReset, ForgetsTheEarlierReading) {
  // The robot may have been moved by hand while stopped. That is not a velocity.
  stapes::test::verifies("SRS-SAF-014");
  SafetyMonitor monitor(make_config());
  ASSERT_EQ(monitor.check(clean_input(0)), StopCause::none);
  SafetyInput bad = clean_input(1);
  bad.target(0) = kNaN;
  ASSERT_EQ(monitor.check(bad), StopCause::non_finite_input);

  const stapes::kinematics::ArmModel model = ArmModel::reference();
  JointVector moved = reference_home();
  moved(5) += 0.2;
  SafetyInput after = clean_input(2);
  after.safety.link_positions_rad = moved;
  after.control.joint_positions_rad = moved;
  after.target = moved;
  after.safety.tracked_tool_pose = forward_kinematics(model, moved);
  ASSERT_TRUE(monitor.try_reset(after));
  EXPECT_EQ(monitor.check(after), StopCause::none);
}

// ---- Priority --------------------------------------------------------------------------------

TEST(SafetyMonitor, ChecksRunInTheDocumentedOrder) {
  // Each case adds one more condition, of higher priority, to the input before it.
  stapes::test::verifies("SRS-SAF-015");
  const SafetyConfig config = make_config();

  SafetyInput input = clean_input(kDebounce - 1);
  input.target(0) += 2.0 * config.max_tracking_error_rad(0);
  const auto debounced_cause = [&config](const SafetyInput& current) {
    SafetyMonitor monitor(config);
    SafetyInput earlier = current;
    for (int i = 0; i < kDebounce; ++i) {
      earlier.now_ticks = i;
      earlier.safety.stamp_ticks = i;
      earlier.control.stamp_ticks = i;
      if (monitor.check(earlier) != StopCause::none) {
        break;
      }
    }
    return monitor.cause();
  };
  EXPECT_EQ(debounced_cause(input), StopCause::tracking_error);

  input.control.joint_positions_rad(1) += 2.0 * config.max_encoder_disagreement_rad(1);
  EXPECT_EQ(debounced_cause(input), StopCause::encoder_disagreement);

  input.safety.tracked_tool_pose.translation() +=
      2.0 * config.max_tracker_disagreement_m * tunnel_direction(config);
  EXPECT_EQ(debounced_cause(input), StopCause::tracker_disagreement);

  input.safety.tracked_tool_pose.translation() += 2.0 * kRadius * sideways(config);
  EXPECT_EQ(debounced_cause(input), StopCause::corridor);

  input.safety.tool_wrench(0) = 2.0 * config.max_tool_force_n;
  EXPECT_EQ(debounced_cause(input), StopCause::tool_force);

  // The immediate checks need no history, except velocity, which needs one earlier reading.
  {
    SafetyMonitor monitor(config);
    SafetyInput earlier = input;
    earlier.now_ticks = 0;
    earlier.safety.stamp_ticks = 0;
    earlier.control.stamp_ticks = 0;
    ASSERT_EQ(monitor.check(earlier), StopCause::none);
    SafetyInput moved = earlier;
    moved.now_ticks = 1;
    moved.safety.stamp_ticks = 1;
    moved.control.stamp_ticks = 1;
    moved.safety.link_positions_rad(3) += 2.0 * config.max_velocity_rad_s(3) * config.period_s;
    EXPECT_EQ(monitor.check(moved), StopCause::joint_velocity);
  }

  input.safety.link_positions_rad(0) = config.arm.joints()[0].upper_rad + 0.1;
  {
    SafetyMonitor monitor(config);
    EXPECT_EQ(monitor.check(input), StopCause::joint_limit);
  }

  input.safety.stamp_ticks = input.now_ticks - config.max_sample_age_ticks - 1;
  {
    SafetyMonitor monitor(config);
    EXPECT_EQ(monitor.check(input), StopCause::stale_data);
  }

  input.target(1) = kNaN;
  {
    SafetyMonitor monitor(config);
    EXPECT_EQ(monitor.check(input), StopCause::non_finite_input);
  }
}

// ---- Structure -------------------------------------------------------------------------------

TEST(SafetyMonitor, ChecksCannotThrow) {
  stapes::test::verifies("SRS-SAF-016");
  SafetyMonitor monitor(make_config());
  static_assert(noexcept(monitor.check(std::declval<const SafetyInput&>())));
  static_assert(noexcept(monitor.try_reset(std::declval<const SafetyInput&>())));
  SUCCEED();
}

TEST(SafetyMonitor, TheModuleIncludesNothingFromTheControllerOrThePlant) {
  // The build already prevents this, since `safety` links neither. This keeps it from being
  // undone by a change to the CMake file going unnoticed.
  stapes::test::verifies("SRS-SAF-016");
  namespace fs = std::filesystem;
  int files = 0;
  for (const fs::directory_entry& entry :
       fs::recursive_directory_iterator(fs::path(STAPES_SOURCE_DIR) / "core" / "safety")) {
    if (!entry.is_regular_file()) {
      continue;
    }
    ++files;
    const std::ifstream file(entry.path());
    std::stringstream contents;
    contents << file.rdbuf();
    const std::string text = contents.str();
    SCOPED_TRACE(entry.path().string());
    EXPECT_EQ(text.find("stapes/control"), std::string::npos);
    EXPECT_EQ(text.find("stapes/plant"), std::string::npos);
    EXPECT_EQ(text.find("stapes::control"), std::string::npos);
    EXPECT_EQ(text.find("stapes::plant"), std::string::npos);
  }
  EXPECT_GT(files, 2);
}

TEST(SafetyMonitor, EveryCauseHasAName) {
  stapes::test::verifies("SRS-SAF-017");
  using stapes::safety::to_string;
  EXPECT_STREQ(to_string(StopCause::none), "none");
  EXPECT_STREQ(to_string(StopCause::non_finite_input), "non_finite_input");
  EXPECT_STREQ(to_string(StopCause::stale_data), "stale_data");
  EXPECT_STREQ(to_string(StopCause::joint_limit), "joint_limit");
  EXPECT_STREQ(to_string(StopCause::joint_velocity), "joint_velocity");
  EXPECT_STREQ(to_string(StopCause::tool_force), "tool_force");
  EXPECT_STREQ(to_string(StopCause::corridor), "corridor");
  EXPECT_STREQ(to_string(StopCause::tracker_disagreement), "tracker_disagreement");
  EXPECT_STREQ(to_string(StopCause::encoder_disagreement), "encoder_disagreement");
  EXPECT_STREQ(to_string(StopCause::tracking_error), "tracking_error");
  // A corrupted value is the point here, and the enum's underlying type can hold it.
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  EXPECT_STREQ(to_string(static_cast<StopCause>(200)), "unknown");
}

}  // namespace
