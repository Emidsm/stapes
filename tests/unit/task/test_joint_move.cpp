#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/task/joint_move.hpp"

namespace {

using stapes::kinematics::JointVector;
using stapes::task::JointMove;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDuration = 4.0;

JointVector from_q() {
  JointVector q;
  q << 0.1, -0.4, 0.7, 0.0, 0.9, -0.2;
  return q;
}

JointVector to_q() {
  JointVector q;
  q << 0.5, -0.3, 0.6, 0.4, 0.2, 0.3;
  return q;
}

TEST(JointMove, StartsAndEndsExactlyAtItsTwoConfigurations) {
  stapes::test::verifies("SRS-KEY-009");
  const JointMove move(from_q(), to_q(), kDuration);
  EXPECT_EQ(move.at(0.0), from_q());
  EXPECT_EQ(move.at(kDuration), to_q());
  // Held outside the move, so a caller that runs past either end gets an end, not a guess.
  EXPECT_EQ(move.at(-1.0), from_q());
  EXPECT_EQ(move.at(kDuration + 1.0), to_q());
}

TEST(JointMove, StartsAndEndsAtRest) {
  // Velocity and acceleration from finite differences, a few microseconds from each end. A
  // linear time scaling would show the full average speed here.
  stapes::test::verifies("SRS-KEY-009");
  const JointMove move(from_q(), to_q(), kDuration);
  constexpr double kStep = 1e-5;
  for (const double end : {0.0, kDuration - (2.0 * kStep)}) {
    const JointVector a = move.at(end);
    const JointVector b = move.at(end + kStep);
    const JointVector c = move.at(end + (2.0 * kStep));
    EXPECT_LT(((b - a) / kStep).cwiseAbs().maxCoeff(), 1e-6) << "at " << end;
    EXPECT_LT(((c - (2.0 * b) + a) / (kStep * kStep)).cwiseAbs().maxCoeff(), 1e-2) << "at " << end;
  }
}

TEST(JointMove, EveryJointMovesMonotonicallyAndInStep) {
  stapes::test::verifies("SRS-KEY-009");
  const JointMove move(from_q(), to_q(), kDuration);
  const JointVector span = to_q() - from_q();
  double previous = 0.0;
  constexpr int kSamples = 400;
  for (int i = 0; i <= kSamples; ++i) {
    const JointVector q = move.at(kDuration * i / kSamples);
    // The fraction of its own span that each joint has covered: the same for all of them.
    const double fraction = (q(0) - from_q()(0)) / span(0);
    for (Eigen::Index joint = 1; joint < q.size(); ++joint) {
      EXPECT_NEAR((q(joint) - from_q()(joint)) / span(joint), fraction, 1e-12);
    }
    EXPECT_GE(fraction, previous);
    previous = fraction;
  }
  // Halfway in time is halfway in space: the time scaling is symmetric.
  EXPECT_NEAR(((move.at(kDuration / 2.0) - from_q()).array() / span.array() - 0.5).abs().maxCoeff(),
              0.0, 1e-12);
}

TEST(JointMove, ReportsThePeakSpeedOfEachJoint) {
  stapes::test::verifies("SRS-KEY-009");
  const JointMove move(from_q(), to_q(), kDuration);
  constexpr double kStep = 1e-6;
  const double mid = kDuration / 2.0;
  const JointVector measured = (move.at(mid + kStep) - move.at(mid - kStep)) / (2.0 * kStep);
  EXPECT_LT((measured.cwiseAbs() - move.peak_speeds_rad_s()).cwiseAbs().maxCoeff(), 1e-6);
  // And nowhere faster than that.
  for (int i = 0; i < 400; ++i) {
    const double t = kDuration * i / 400.0;
    const JointVector speed = (move.at(t + kStep) - move.at(t)) / kStep;
    EXPECT_TRUE((speed.cwiseAbs().array() <= move.peak_speeds_rad_s().array() + 1e-6).all());
  }
}

TEST(JointMove, RejectsNonFiniteEndsAndABadDuration) {
  stapes::test::verifies("SRS-KEY-009");
  JointVector bad = from_q();
  bad(2) = kNaN;
  EXPECT_THROW(JointMove(bad, to_q(), kDuration), std::invalid_argument);
  EXPECT_THROW(JointMove(from_q(), bad, kDuration), std::invalid_argument);
  for (const double duration : {0.0, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(JointMove(from_q(), to_q(), duration), std::invalid_argument) << duration;
  }
}

}  // namespace
