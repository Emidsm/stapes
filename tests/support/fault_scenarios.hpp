#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

#include "stapes/base/state_hash.hpp"
#include "stapes/sim/keyhole_simulation.hpp"

namespace stapes::test {

/// Long enough for the reference feed to finish and settle.
inline constexpr int kRunTicks = 3200;

/// What actually happened to the tool, from the true state. No running code can see these.
struct RunResult {
  double max_radial_m{0.0};
  double max_axis_offset_m{0.0};
  double final_depth_m{0.0};
  safety::StopCause cause{safety::StopCause::none};
  std::optional<std::int64_t> stop_tick;
};

inline void record(const sim::KeyholeSimulation& sim, RunResult& result) {
  const math::Pose tool = sim.true_tool_pose();
  const task::KeyholeTrajectory& tunnel = sim.trajectory();
  result.max_radial_m = std::max(result.max_radial_m, tunnel.radial_distance(tool.translation()));
  result.max_axis_offset_m =
      std::max(result.max_axis_offset_m, task::axis_distance(tool, tunnel.entry()));
  result.final_depth_m = tunnel.depth_of(tool.translation());
  result.cause = sim.monitor().cause();
  result.stop_tick = sim.stop_tick();
}

inline RunResult run_scenario(const sim::KeyholeScenario& scenario, int ticks) {
  sim::KeyholeSimulation sim(scenario);
  RunResult result;
  for (int i = 0; i < ticks; ++i) {
    sim.step();
    record(sim, result);
  }
  return result;
}

/// Runs the feed against a force opposing it, building up over 0.4 s as a drill meets bone.
inline RunResult run_with_thrust(const sim::KeyholeScenario& scenario, double force_n) {
  constexpr int kRampTicks = 400;
  sim::KeyholeSimulation sim(scenario);
  RunResult result;
  for (int i = 0; i < kRunTicks; ++i) {
    const double ramp = std::min(1.0, static_cast<double>(i) / kRampTicks);
    math::Wrench wrench = math::Wrench::Zero();
    wrench.head<3>() = -scenario.trajectory.direction() * (force_n * ramp);
    sim.set_external_wrench(wrench);
    sim.step();
    record(sim, result);
  }
  return result;
}

/// A run with a fault, reduced to one hash. Includes the monitor's decision and when it was taken.
inline std::uint64_t fault_workload_hash(std::uint64_t seed, int ticks) {
  sim::KeyholeScenario scenario = sim::KeyholeScenario::reference(seed);
  scenario.fault = faults::Fault{faults::FaultKind::frozen_motor_encoder, 500, 1, 0};
  sim::KeyholeSimulation sim(scenario);
  base::StateHash hash;
  for (int i = 0; i < ticks; ++i) {
    sim.step();
    sim.add_to_hash(hash);
  }
  return hash.value();
}

}  // namespace stapes::test
