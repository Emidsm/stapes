#pragma once

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include "stapes/base/random.hpp"
#include "stapes/kinematics/arm_model.hpp"

namespace stapes::test {

/// Runs `property` on many generated cases. Each case has its own random stream, so
/// the seed and case number printed on failure reproduce that case on its own.
/// Stops at the first failing case.
template <typename Property>
void for_all_cases(std::uint64_t seed, int case_count, Property property) {
  for (int case_index = 0; case_index < case_count; ++case_index) {
    SCOPED_TRACE("seed=" + std::to_string(seed) + " case=" + std::to_string(case_index));
    base::Rng rng(seed, static_cast<std::uint64_t>(case_index));
    property(rng);
    if (::testing::Test::HasFailure()) {
      return;
    }
  }
}

/// A uniformly random configuration at least `margin_rad` inside every joint limit.
inline kinematics::JointVector random_configuration(const kinematics::ArmModel& model,
                                                    base::Rng& rng, double margin_rad = 0.0) {
  kinematics::JointVector q;
  Eigen::Index row = 0;
  for (const kinematics::RevoluteJoint& joint : model.joints()) {
    q(row++) = rng.uniform(joint.lower_rad + margin_rad, joint.upper_rad - margin_rad);
  }
  return q;
}

}  // namespace stapes::test
