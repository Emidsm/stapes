// Prints the hash of a seeded workload, so a separate process can be compared.

#include <iomanip>
#include <iostream>

#include "kinematics_workload.hpp"

int main() {
  constexpr std::uint64_t kSeed = 42;
  constexpr int kCases = 500;
  std::cout << std::hex << std::setw(16) << std::setfill('0')
            << stapes::test::kinematics_workload_hash(kSeed, kCases) << '\n';
  return 0;
}
