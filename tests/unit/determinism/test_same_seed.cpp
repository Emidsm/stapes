#include <gtest/gtest.h>

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

}  // namespace
