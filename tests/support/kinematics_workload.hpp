#pragma once

#include <cstdint>

#include "property.hpp"
#include "stapes/base/random.hpp"
#include "stapes/base/state_hash.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"

namespace stapes::test {

/// Random draws, forward kinematics and inverse kinematics, reduced to one hash.
/// Any change in a single bit of any result changes the hash.
inline std::uint64_t kinematics_workload_hash(std::uint64_t seed, int case_count) {
  const kinematics::ArmModel model = kinematics::ArmModel::reference();
  base::Rng rng(seed);
  base::StateHash hash;

  for (int i = 0; i < case_count; ++i) {
    const kinematics::JointVector goal = random_configuration(model, rng);
    const kinematics::JointVector seed_q = random_configuration(model, rng);
    const kinematics::IkResult result =
        kinematics::solve_ik(model, kinematics::forward_kinematics(model, goal), seed_q);

    hash.add(std::span<const double>(result.q.data(), kinematics::kJointCount));
    hash.add(static_cast<std::uint64_t>(result.iterations));
    hash.add(result.position_error_m);
    hash.add(rng.gaussian());
  }
  return hash.value();
}

}  // namespace stapes::test
