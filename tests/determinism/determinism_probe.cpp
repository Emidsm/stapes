// Prints the hash of a seeded workload, so a separate process can be compared.

#include <iomanip>
#include <iostream>

#include "closed_loop_workload.hpp"
#include "fault_scenarios.hpp"
#include "kinematics_workload.hpp"

int main() {
  constexpr std::uint64_t kSeed = 42;
  constexpr int kCases = 500;
  constexpr int kTicks = 2000;
  std::cout << std::hex << std::setfill('0') << std::setw(16)
            << stapes::test::kinematics_workload_hash(kSeed, kCases) << '-' << std::setw(16)
            << stapes::test::closed_loop_workload_hash(kSeed, kTicks) << '-' << std::setw(16)
            << stapes::test::fault_workload_hash(kSeed, kTicks) << '\n';
  return 0;
}
