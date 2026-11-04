#include <benchmark/benchmark.h>

#include "stapes/control/joint_controller.hpp"
#include "stapes/dynamics/rigid_body_dynamics.hpp"
#include "stapes/plant/simulated_arm.hpp"

namespace {

using stapes::dynamics::DynamicModel;
using stapes::kinematics::JointVector;
using stapes::kinematics::reference_home;

void BM_InverseDynamics(benchmark::State& state) {
  const DynamicModel model = DynamicModel::reference();
  const JointVector q = reference_home();
  const JointVector qd = JointVector::Constant(0.5);
  for (auto _ : state) {
    benchmark::DoNotOptimize(stapes::dynamics::inverse_dynamics(model, q, qd, qd));
  }
}
BENCHMARK(BM_InverseDynamics);

void BM_MassMatrix(benchmark::State& state) {
  const DynamicModel model = DynamicModel::reference();
  const JointVector q = reference_home();
  for (auto _ : state) {
    benchmark::DoNotOptimize(stapes::dynamics::mass_matrix(model, q));
  }
}
BENCHMARK(BM_MassMatrix);

// One control period: one controller update and ten physics steps. The real-time budget for
// this at 1 kHz is 1 ms.
void BM_ClosedLoopTick(benchmark::State& state) {
  const stapes::plant::SimulatedArmConfig config = stapes::plant::SimulatedArmConfig::reference(1);
  stapes::plant::SimulatedArm arm(config);
  stapes::control::JointController controller(config.model, stapes::control::reference_gains(),
                                              stapes::control::reference_gearbox_estimates(), 1e-3);
  const JointVector target = reference_home();
  for (auto _ : state) {
    arm.command_torques(controller.update(target, arm.read_sensors().joint_positions_rad));
    arm.tick();
  }
}
BENCHMARK(BM_ClosedLoopTick);

}  // namespace

BENCHMARK_MAIN();
