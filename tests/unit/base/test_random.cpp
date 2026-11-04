#include <array>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/base/random.hpp"

namespace {

using stapes::base::Rng;
using stapes::base::SplitMix64;
using stapes::base::Xoshiro256pp;

// Reference values come from independent implementations of the published algorithms,
// in tools/reference_vectors.py. They pin the exact output: any change to the
// generators, on any compiler, fails here.
TEST(SplitMix64, MatchesReferenceVector) {
  stapes::test::verifies("SRS-DET-001");
  SplitMix64 generator(1234567);
  const std::array<std::uint64_t, 5> expected{6457827717110365317ULL, 3203168211198807973ULL,
                                              9817491932198370423ULL, 4593380528125082431ULL,
                                              16408922859458223821ULL};
  for (const std::uint64_t value : expected) {
    EXPECT_EQ(generator.next(), value);
  }
}

TEST(Xoshiro256pp, MatchesReferenceVector) {
  stapes::test::verifies("SRS-DET-001");
  Xoshiro256pp generator(Xoshiro256pp::State{1, 2, 3, 4});
  const std::array<std::uint64_t, 6> expected{41943041ULL,
                                              58720359ULL,
                                              3588806011781223ULL,
                                              3591011842654386ULL,
                                              9228616714210784205ULL,
                                              9973669472204895162ULL};
  for (const std::uint64_t value : expected) {
    EXPECT_EQ(generator.next(), value);
  }
}

TEST(Rng, SameSeedAndStreamGiveTheSameSequence) {
  stapes::test::verifies("SRS-DET-002");
  Rng first(7, 3);
  Rng second(7, 3);
  for (int i = 0; i < 1000; ++i) {
    ASSERT_EQ(first.next_u64(), second.next_u64());
  }
}

TEST(Rng, DifferentSeedsGiveDifferentSequences) {
  stapes::test::verifies("SRS-DET-002");
  Rng first(7);
  Rng second(8);
  EXPECT_NE(first.next_u64(), second.next_u64());
}

TEST(Rng, StreamsAreIndependent) {
  stapes::test::verifies("SRS-DET-003");
  Rng observed(7, 1);
  Rng reference(7, 1);
  Rng other_stream(7, 2);

  // Drawing from another stream must not disturb this one.
  for (int i = 0; i < 1000; ++i) {
    static_cast<void>(other_stream.next_u64());
    ASSERT_EQ(observed.next_u64(), reference.next_u64());
  }
}

TEST(Rng, StreamsProduceDifferentSequences) {
  stapes::test::verifies("SRS-DET-003");
  // Includes the pairs where seed and stream are swapped, which a plain XOR of the two
  // would map to the same state.
  EXPECT_NE(Rng(7, 1).next_u64(), Rng(7, 2).next_u64());
  EXPECT_NE(Rng(7, 1).next_u64(), Rng(1, 7).next_u64());
  EXPECT_NE(Rng(0, 0).next_u64(), Rng(0, 1).next_u64());
}

TEST(Rng, UniformStaysInItsHalfOpenRange) {
  stapes::test::verifies("SRS-DET-004");
  Rng rng(11);
  for (int i = 0; i < 100'000; ++i) {
    const double unit = rng.uniform01();
    ASSERT_GE(unit, 0.0);
    ASSERT_LT(unit, 1.0);
    const double ranged = rng.uniform(-2.0, 3.0);
    ASSERT_GE(ranged, -2.0);
    ASSERT_LT(ranged, 3.0);
  }
}

TEST(Rng, UniformHasTheExpectedMoments) {
  stapes::test::verifies("SRS-DET-004");
  Rng rng(13);
  constexpr int kSamples = 1'000'000;
  double sum = 0.0;
  double sum_squares = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double x = rng.uniform01();
    sum += x;
    sum_squares += x * x;
  }
  const double mean = sum / kSamples;
  const double variance = (sum_squares / kSamples) - (mean * mean);
  // Five standard errors of the estimates: 0.0015 and 0.0005 for this sample size.
  EXPECT_NEAR(mean, 0.5, 0.0015);
  EXPECT_NEAR(variance, 1.0 / 12.0, 0.0005);
}

TEST(Rng, GaussianHasTheExpectedMoments) {
  stapes::test::verifies("SRS-DET-004");
  Rng rng(17);
  constexpr int kSamples = 1'000'000;
  double sum = 0.0;
  double sum_squares = 0.0;
  double sum_cubes = 0.0;
  double sum_fourths = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const double x = rng.gaussian();
    const double x2 = x * x;
    sum += x;
    sum_squares += x2;
    sum_cubes += x2 * x;
    sum_fourths += x2 * x2;
  }
  // Five standard errors. A sampler that gets the variance right with the wrong shape
  // fails on the third or fourth moment.
  EXPECT_NEAR(sum / kSamples, 0.0, 0.005);
  EXPECT_NEAR(sum_squares / kSamples, 1.0, 0.0071);
  EXPECT_NEAR(sum_cubes / kSamples, 0.0, 0.0195);
  EXPECT_NEAR(sum_fourths / kSamples, 3.0, 0.049);
}

TEST(Rng, GaussianFollowsThePolarMethodStepByStep) {
  // Each accepted pair of uniforms yields two samples: x * scale, then y * scale.
  // Re-derived here from the raw uniform draws, so it also shows the second sample of
  // the pair is used rather than thrown away.
  stapes::test::verifies("SRS-DET-004");
  Rng sampler(23);
  Rng uniforms(23);
  for (int pair = 0; pair < 1000; ++pair) {
    double x = 0.0;
    double y = 0.0;
    double radius_squared = 0.0;
    do {
      x = uniforms.uniform(-1.0, 1.0);
      y = uniforms.uniform(-1.0, 1.0);
      radius_squared = (x * x) + (y * y);
    } while (radius_squared >= 1.0 || radius_squared == 0.0);
    const double scale = std::sqrt(-2.0 * std::log(radius_squared) / radius_squared);

    ASSERT_EQ(sampler.gaussian(), x * scale);
    ASSERT_EQ(sampler.gaussian(), y * scale);
  }
}

TEST(Rng, GaussianAppliesMeanAndStandardDeviation) {
  stapes::test::verifies("SRS-DET-004");
  Rng scaled(19);
  Rng unit(19);
  for (int i = 0; i < 1000; ++i) {
    ASSERT_DOUBLE_EQ(scaled.gaussian(5.0, 2.0), 5.0 + (2.0 * unit.gaussian()));
  }
}

}  // namespace
