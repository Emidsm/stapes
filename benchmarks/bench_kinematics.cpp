#include <benchmark/benchmark.h>

#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"

namespace {

using stapes::kinematics::ArmModel;
using stapes::kinematics::JointVector;
using stapes::kinematics::reference_home;

void BM_ForwardKinematics(benchmark::State& state) {
  const ArmModel model = ArmModel::reference();
  const JointVector q = reference_home();
  for (auto _ : state) {
    benchmark::DoNotOptimize(stapes::kinematics::forward_kinematics(model, q));
  }
}
BENCHMARK(BM_ForwardKinematics);

void BM_GeometricJacobian(benchmark::State& state) {
  const ArmModel model = ArmModel::reference();
  const JointVector q = reference_home();
  for (auto _ : state) {
    benchmark::DoNotOptimize(stapes::kinematics::geometric_jacobian(model, q));
  }
}
BENCHMARK(BM_GeometricJacobian);

void BM_InverseKinematicsNearbySeed(benchmark::State& state) {
  const ArmModel model = ArmModel::reference();
  JointVector goal = reference_home();
  goal(0) += 0.05;
  goal(2) -= 0.05;
  const stapes::math::Pose target = stapes::kinematics::forward_kinematics(model, goal);
  const JointVector seed = reference_home();
  for (auto _ : state) {
    benchmark::DoNotOptimize(stapes::kinematics::solve_ik(model, target, seed));
  }
}
BENCHMARK(BM_InverseKinematicsNearbySeed);

}  // namespace

BENCHMARK_MAIN();
