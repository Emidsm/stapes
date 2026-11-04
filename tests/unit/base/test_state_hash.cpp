#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/base/state_hash.hpp"

namespace {

using stapes::base::StateHash;

TEST(StateHash, EmptyHashIsTheFnvOffsetBasis) {
  stapes::test::verifies("SRS-DET-005");
  EXPECT_EQ(StateHash{}.value(), 0xCBF29CE484222325ULL);
}

TEST(StateHash, MatchesFnv1aReferenceForAKnownInput) {
  // Reference: FNV-1a 64 of the eight bytes 01 00 00 00 00 00 00 00, computed by an
  // independent implementation in tools/reference_vectors.py.
  stapes::test::verifies("SRS-DET-005");
  StateHash hash;
  hash.add(std::uint64_t{1});
  EXPECT_EQ(hash.value(), 0x89CD31291D2AEFA4ULL);
}

TEST(StateHash, SeesTheSmallestPossibleDoubleChange) {
  stapes::test::verifies("SRS-DET-005");
  const double value = 1.0;
  StateHash first;
  StateHash second;
  first.add(value);
  second.add(std::nextafter(value, 2.0));
  EXPECT_NE(first.value(), second.value());
}

TEST(StateHash, DependsOnOrder) {
  stapes::test::verifies("SRS-DET-005");
  const std::array<double, 2> forward{1.0, 2.0};
  const std::array<double, 2> backward{2.0, 1.0};
  StateHash first;
  StateHash second;
  first.add(forward);
  second.add(backward);
  EXPECT_NE(first.value(), second.value());
}

}  // namespace
