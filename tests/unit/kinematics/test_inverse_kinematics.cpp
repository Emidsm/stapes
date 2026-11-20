#include <algorithm>
#include <limits>

#include <gtest/gtest.h>

#include "property.hpp"
#include "requirement.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"

namespace {

using stapes::kinematics::ArmModel;
using stapes::kinematics::forward_kinematics;
using stapes::kinematics::IkOptions;
using stapes::kinematics::IkResult;
using stapes::kinematics::IkStatus;
using stapes::kinematics::JointVector;
using stapes::kinematics::reference_home;
using stapes::kinematics::solve_ik;
using stapes::kinematics::solve_ik_multistart;
using stapes::math::Pose;

const ArmModel kModel = ArmModel::reference();

JointVector perturbed(const JointVector& q, double amount, stapes::base::Rng& rng) {
  JointVector result = q;
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    result(i) += rng.uniform(-amount, amount);
  }
  return kModel.clamp_to_limits(result);
}

void expect_reaches(const IkResult& result, const Pose& target, const IkOptions& options) {
  ASSERT_TRUE(result.converged()) << "position error " << result.position_error_m
                                  << " orientation error " << result.orientation_error_rad;

  // Checked with forward kinematics directly, not the solver's own report of its error.
  const Pose reached = forward_kinematics(kModel, result.q);
  EXPECT_LE((reached.translation() - target.translation()).norm(), options.position_tolerance_m);
  EXPECT_LE(stapes::math::orientation_error(target.linear(), reached.linear()).norm(),
            options.orientation_tolerance_rad);
}

TEST(InverseKinematics, ReturnsImmediatelyWhenAlreadyAtTarget) {
  stapes::test::verifies("SRS-KIN-003");
  const JointVector q = reference_home();
  const IkResult result = solve_ik(kModel, forward_kinematics(kModel, q), q);
  EXPECT_TRUE(result.converged());
  EXPECT_EQ(result.iterations, 0);
  EXPECT_EQ(result.q, q);
}

TEST(InverseKinematics, ReachesPosesFromANearbySeed) {
  stapes::test::verifies("SRS-KIN-003");
  const IkOptions options;
  stapes::test::for_all_cases(401, 5000, [&options](stapes::base::Rng& rng) {
    const JointVector goal = stapes::test::random_configuration(kModel, rng);
    const Pose target = forward_kinematics(kModel, goal);
    const JointVector seed = perturbed(goal, 0.2, rng);
    expect_reaches(solve_ik(kModel, target, seed, options), target, options);
  });
}

TEST(InverseKinematics, ReachesEveryPoseWithinHalfARadianOfHome) {
  stapes::test::verifies("SRS-KIN-003");
  const IkOptions options;
  stapes::test::for_all_cases(409, 5000, [&options](stapes::base::Rng& rng) {
    const Pose target = forward_kinematics(kModel, perturbed(reference_home(), 0.5, rng));
    expect_reaches(solve_ik(kModel, target, reference_home(), options), target, options);
  });
}

TEST(MultistartInverseKinematics, ReachesNearlyAllPosesFromHome) {
  stapes::test::verifies("SRS-KIN-009");
  constexpr int kCases = 5000;
  int reached = 0;
  int reached_by_one_start = 0;
  for (int i = 0; i < kCases; ++i) {
    stapes::base::Rng rng(403, static_cast<std::uint64_t>(i));
    const Pose target = forward_kinematics(kModel, stapes::test::random_configuration(kModel, rng));
    reached += solve_ik_multistart(kModel, target, reference_home(), rng).converged() ? 1 : 0;
    reached_by_one_start += solve_ik(kModel, target, reference_home()).converged() ? 1 : 0;
  }
  RecordProperty("reached_multistart", reached);
  RecordProperty("reached_one_start", reached_by_one_start);
  RecordProperty("cases", kCases);
  EXPECT_GE(reached, kCases * 99 / 100) << "reached " << reached << " of " << kCases;
  // Restarts must be what makes the difference, or this test proves nothing about them.
  EXPECT_GT(reached, reached_by_one_start);
}

TEST(MultistartInverseKinematics, StopsAtTheFirstSolution) {
  stapes::test::verifies("SRS-KIN-009");
  const JointVector q = reference_home();
  stapes::base::Rng rng(411);
  stapes::base::Rng untouched(411);
  const IkResult result = solve_ik_multistart(kModel, forward_kinematics(kModel, q), q, rng);
  EXPECT_TRUE(result.converged());
  EXPECT_EQ(result.iterations, 0);
  EXPECT_EQ(rng.next_u64(), untouched.next_u64()) << "drew random numbers it did not need";
}

TEST(MultistartInverseKinematics, IsDeterministicForAGivenGeneratorState) {
  stapes::test::verifies("SRS-DET-002");
  Pose target = Pose::Identity();
  target.translation() = Eigen::Vector3d(2.0, 0.0, 0.3);
  stapes::base::Rng first_rng(413);
  stapes::base::Rng second_rng(413);
  const IkResult first = solve_ik_multistart(kModel, target, reference_home(), first_rng, 8);
  const IkResult second = solve_ik_multistart(kModel, target, reference_home(), second_rng, 8);
  EXPECT_EQ(first.q, second.q);
  EXPECT_EQ(first.iterations, second.iterations);
}

TEST(MultistartInverseKinematics, ReturnsTheClosestAttemptWhenUnreachable) {
  stapes::test::verifies("SRS-KIN-005");
  Pose target = Pose::Identity();
  target.translation() = Eigen::Vector3d(2.0, 0.0, 0.3);

  stapes::base::Rng single_start_rng(415);
  stapes::base::Rng many_starts_rng(415);
  const IkResult one = solve_ik_multistart(kModel, target, reference_home(), single_start_rng, 1);
  const IkResult many = solve_ik_multistart(kModel, target, reference_home(), many_starts_rng, 16);

  EXPECT_FALSE(many.converged());
  EXPECT_TRUE(kModel.within_limits(many.q));
  EXPECT_GT(many.iterations, one.iterations);
  EXPECT_LE(many.position_error_m, one.position_error_m);
}

TEST(MultistartInverseKinematics, RejectsNonFiniteInputWithoutRestarting) {
  stapes::test::verifies("SRS-KIN-005");
  Pose target = forward_kinematics(kModel, reference_home());
  target.translation().y() = std::numeric_limits<double>::infinity();
  stapes::base::Rng rng(417);
  stapes::base::Rng untouched(417);
  EXPECT_EQ(solve_ik_multistart(kModel, target, reference_home(), rng).status,
            IkStatus::invalid_input);
  EXPECT_EQ(rng.next_u64(), untouched.next_u64());
}

TEST(InverseKinematics, AlwaysStaysWithinJointLimits) {
  stapes::test::verifies("SRS-KIN-004");
  stapes::test::for_all_cases(405, 5000, [](stapes::base::Rng& rng) {
    // Any target, reachable or not, and any seed, even one outside the limits.
    Pose target = forward_kinematics(kModel, stapes::test::random_configuration(kModel, rng));
    target.translation() +=
        Eigen::Vector3d{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)};
    JointVector seed;
    for (Eigen::Index i = 0; i < seed.size(); ++i) {
      seed(i) = rng.uniform(-6.0, 6.0);
    }
    EXPECT_TRUE(kModel.within_limits(solve_ik(kModel, target, seed).q));
  });
}

TEST(InverseKinematics, NeverClaimsSuccessOnAnUnreachableTarget) {
  stapes::test::verifies("SRS-KIN-005");
  stapes::test::for_all_cases(407, 2000, [](stapes::base::Rng& rng) {
    // The reference arm cannot reach further than about 0.75 m from the shoulder, so a
    // target 2 m out is unreachable from every seed.
    Pose target = Pose::Identity();
    target.translation() =
        Eigen::Vector3d{rng.gaussian(), rng.gaussian(), rng.gaussian()}.normalized() * 2.0;
    target.translation().z() += 0.30;
    const IkResult result =
        solve_ik(kModel, target, stapes::test::random_configuration(kModel, rng));
    EXPECT_FALSE(result.converged());
    EXPECT_GT(result.position_error_m, 1.0);
  });
}

TEST(InverseKinematics, ReportsTheErrorItActuallyAchieved) {
  stapes::test::verifies("SRS-KIN-005");
  Pose target = Pose::Identity();
  target.translation() = Eigen::Vector3d(3.0, 0.0, 0.3);
  const IkResult result = solve_ik(kModel, target, reference_home());
  const Pose reached = forward_kinematics(kModel, result.q);
  EXPECT_DOUBLE_EQ(result.position_error_m, (reached.translation() - target.translation()).norm());
}

TEST(InverseKinematics, RespectsTheIterationBudget) {
  stapes::test::verifies("SRS-KIN-005");
  IkOptions options;
  options.max_iterations = 2;
  JointVector far_goal = reference_home();
  far_goal(0) += 1.5;
  far_goal(2) += 1.0;
  const IkResult result =
      solve_ik(kModel, forward_kinematics(kModel, far_goal), reference_home(), options);
  EXPECT_EQ(result.status, IkStatus::max_iterations);
  EXPECT_EQ(result.iterations, 2);
}

TEST(InverseKinematics, LimitsTheChangeInOneIteration) {
  // A far goal makes the undamped step large, so the limit is what bounds it.
  stapes::test::verifies("SRS-KIN-010");
  IkOptions options;
  options.max_iterations = 1;
  options.max_step_rad = 0.01;
  JointVector far_goal = reference_home();
  far_goal(0) += 1.5;
  far_goal(2) += 1.0;
  const IkResult result =
      solve_ik(kModel, forward_kinematics(kModel, far_goal), reference_home(), options);

  const double largest_change = (result.q - reference_home()).cwiseAbs().maxCoeff();
  EXPECT_GT(largest_change, 0.0);
  EXPECT_LE(largest_change, options.max_step_rad * (1.0 + 1e-12));
}

TEST(InverseKinematics, ReportsStalledRatherThanRunningOutOfBudget) {
  stapes::test::verifies("SRS-KIN-010");
  IkOptions options;
  options.max_iterations = 100000;
  Pose target = Pose::Identity();
  target.translation() = Eigen::Vector3d(2.0, 0.0, 0.3);
  const IkResult result = solve_ik(kModel, target, reference_home(), options);
  EXPECT_EQ(result.status, IkStatus::stalled);
  EXPECT_LT(result.iterations, 1000);
}

TEST(InverseKinematics, GivesUpWhenProgressBecomesNegligible) {
  // Failed attempts are where multistart spends its time. On this seeded set, 209 of
  // 5000 attempts use the whole budget without the negligible-progress stop, and 114
  // with it. The threshold sits between the two.
  stapes::test::verifies("SRS-KIN-010");
  constexpr int kCases = 5000;
  constexpr int kMaxExhausted = 160;
  int exhausted = 0;
  for (int i = 0; i < kCases; ++i) {
    stapes::base::Rng rng(77, static_cast<std::uint64_t>(i));
    Pose target = forward_kinematics(kModel, stapes::test::random_configuration(kModel, rng));
    target.translation() +=
        Eigen::Vector3d{rng.gaussian(), rng.gaussian(), rng.gaussian()}.normalized() * 0.1;
    const JointVector seed = stapes::test::random_configuration(kModel, rng);
    exhausted += solve_ik(kModel, target, seed).status == IkStatus::max_iterations ? 1 : 0;
  }
  RecordProperty("exhausted_budget", exhausted);
  EXPECT_LE(exhausted, kMaxExhausted);
}

TEST(InverseKinematics, RejectsNonFiniteInput) {
  stapes::test::verifies("SRS-KIN-005");
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  JointVector bad_seed = reference_home();
  bad_seed(1) = kNaN;
  const Pose valid_target = forward_kinematics(kModel, reference_home());
  const IkResult from_bad_seed = solve_ik(kModel, valid_target, bad_seed);
  EXPECT_EQ(from_bad_seed.status, IkStatus::invalid_input);
  EXPECT_TRUE(kModel.within_limits(from_bad_seed.q));

  Pose bad_target = valid_target;
  bad_target.translation().x() = kNaN;
  EXPECT_EQ(solve_ik(kModel, bad_target, reference_home()).status, IkStatus::invalid_input);
}

}  // namespace
