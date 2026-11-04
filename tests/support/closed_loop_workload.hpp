#pragma once

#include <cstdint>

#include "stapes/base/state_hash.hpp"
#include "stapes/control/joint_controller.hpp"
#include "stapes/plant/simulated_arm.hpp"

namespace stapes::test {

/// One control period: read through the hardware boundary, compute, command.
inline void control_step(hardware::RobotInterface& robot, control::JointController& controller,
                         const kinematics::JointVector& target) {
  robot.command_torques(controller.update(target, robot.read_sensors().joint_positions_rad));
}

/// The reference arm under closed-loop control through a step, reduced to one hash. Every
/// model, every noise source and the controller all feed into the result.
inline std::uint64_t closed_loop_workload_hash(std::uint64_t seed, int ticks) {
  const plant::SimulatedArmConfig config = plant::SimulatedArmConfig::reference(seed);
  plant::SimulatedArm arm(config);
  control::JointController controller(config.model, control::reference_gains(),
                                      control::reference_gearbox_estimates(), 1e-3);
  kinematics::JointVector target = config.initial_q;
  target.array() += 0.05;

  base::StateHash hash;
  for (int i = 0; i < ticks; ++i) {
    control_step(arm, controller, target);
    arm.tick();
    arm.add_to_hash(hash);
  }
  return hash.value();
}

}  // namespace stapes::test
