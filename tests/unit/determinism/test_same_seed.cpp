#include <gtest/gtest.h>

#include "closed_loop_workload.hpp"
#include "fault_scenarios.hpp"
#include "kinematics_workload.hpp"
#include "requirement.hpp"

namespace {

using stapes::test::kinematics_workload_hash;

TEST(Determinism, SameSeedGivesTheSameHash) {
  stapes::test::verifies("SRS-DET-002");
  EXPECT_EQ(kinematics_workload_hash(42, 500), kinematics_workload_hash(42, 500));
}

TEST(Determinism, DifferentSeedGivesADifferentHash) {
  // Without this, a workload that ignored its seed would pass the test above.
  stapes::test::verifies("SRS-DET-002");
  EXPECT_NE(kinematics_workload_hash(42, 500), kinematics_workload_hash(43, 500));
}

TEST(Determinism, ClosedLoopRunsAreBitIdentical) {
  stapes::test::verifies("SRS-DET-006");
  EXPECT_EQ(stapes::test::closed_loop_workload_hash(42, 2000),
            stapes::test::closed_loop_workload_hash(42, 2000));
}

TEST(Determinism, RunsWithAFaultAndAStopAreBitIdentical) {
  stapes::test::verifies("SRS-DET-007");
  EXPECT_EQ(stapes::test::fault_workload_hash(42, 2000),
            stapes::test::fault_workload_hash(42, 2000));
}

TEST(Determinism, ADifferentSeedChangesARunWithAFault) {
  stapes::test::verifies("SRS-DET-007");
  EXPECT_NE(stapes::test::fault_workload_hash(42, 2000),
            stapes::test::fault_workload_hash(43, 2000));
}

TEST(Determinism, ADifferentSeedChangesTheClosedLoopRun) {
  stapes::test::verifies("SRS-DET-006");
  EXPECT_NE(stapes::test::closed_loop_workload_hash(42, 2000),
            stapes::test::closed_loop_workload_hash(43, 2000));
}

}  // namespace
