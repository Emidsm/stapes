#include "stapes/plant/simulated_arm.hpp"

#include <chrono>
#include <cstddef>
#include <numbers>
#include <span>
#include <stdexcept>
#include <utility>

#include "stapes/dynamics/rigid_body_dynamics.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"

namespace stapes::plant {

namespace {

using kinematics::JointVector;

// Each noise source has a fixed stream, so adding a sensor never changes another's noise.
constexpr std::uint64_t kEncoderStreamBase = 100;
constexpr std::uint64_t kForceSensorStream = 200;

// Runs first in the constructor's initialiser list, so nothing is built from a bad config.
dynamics::DynamicModel validated_model(const SimulatedArmConfig& config) {
  const base::SimDuration zero = base::SimDuration::zero();
  if (config.physics_step <= zero || config.control_period <= zero ||
      config.control_period % config.physics_step != zero) {
    throw std::invalid_argument("control period must be a whole multiple of the physics step");
  }
  if (!config.initial_q.allFinite()) {
    throw std::invalid_argument("initial configuration must be finite");
  }
  return config.model;
}

template <std::size_t... Index>
std::array<Encoder, sizeof...(Index)> make_encoders(const SimulatedArmConfig& config,
                                                    std::index_sequence<Index...> /*unused*/) {
  return {Encoder(config.encoder, base::Rng(config.seed, kEncoderStreamBase + Index))...};
}

double seconds(base::SimDuration duration) {
  return std::chrono::duration<double>(duration).count();
}

}  // namespace

SimulatedArmConfig SimulatedArmConfig::reference(std::uint64_t seed) {
  using std::chrono::microseconds;
  using std::chrono::milliseconds;
  constexpr double kCountsPerTurn = 1048576.0;  // 2^20.
  return SimulatedArmConfig{
      dynamics::DynamicModel::reference(),
      reference_actuators(),
      EncoderParams{2.0 * std::numbers::pi / kCountsPerTurn, 2e-6},
      ForceSensorParams{0.05, 0.002, 0.01, 0.0005, 0.0},
      microseconds(100),
      milliseconds(1),
      kinematics::reference_home(),
      seed,
  };
}

SimulatedArm::SimulatedArm(const SimulatedArmConfig& config)
    : model_(validated_model(config)),
      actuators_(config.actuators),
      encoders_(make_encoders(config, std::make_index_sequence<kinematics::kJointCount>{})),
      force_sensor_(config.force_sensor, base::Rng(config.seed, kForceSensorStream)),
      clock_(config.control_period),
      physics_steps_per_tick_(static_cast<int>(config.control_period / config.physics_step)),
      physics_step_s_(seconds(config.physics_step)),
      control_period_s_(seconds(config.control_period)),
      state_{config.initial_q, JointVector::Zero(), config.initial_q, JointVector::Zero()} {
  sample_sensors();
}

void SimulatedArm::command_torques(const JointVector& torques_nm) {
  for (std::size_t i = 0; i < actuators_.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    applied_torques_(row) = saturate_torque(torques_nm(row), actuators_[i].max_torque_nm);
  }
}

void SimulatedArm::tick() noexcept {
  for (int i = 0; i < physics_steps_per_tick_; ++i) {
    step_physics(physics_step_s_);
  }
  clock_.advance();
  sample_sensors();
}

void SimulatedArm::step_physics(double step_s) noexcept {
  JointVector gear;
  JointVector motor_accel;
  for (std::size_t i = 0; i < actuators_.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    const ActuatorParams& actuator = actuators_[i];
    gear(row) = gear_torque(actuator.gear, state_.motor_angle(row) - state_.q(row),
                            state_.motor_velocity(row) - state_.qd(row));
    const double friction = friction_torque(actuator.friction, state_.motor_velocity(row));
    motor_accel(row) = (applied_torques_(row) - friction - gear(row)) / actuator.rotor_inertia_kgm2;
  }

  const JointVector environment =
      kinematics::geometric_jacobian(model_.arm(), state_.q).transpose() * external_wrench_;
  const JointVector link_accel =
      dynamics::forward_dynamics(model_, state_.q, state_.qd, gear + environment);

  // Semi-implicit Euler: new velocities move the positions.
  state_.qd += link_accel * step_s;
  state_.motor_velocity += motor_accel * step_s;
  state_.q += state_.qd * step_s;
  state_.motor_angle += state_.motor_velocity * step_s;
}

void SimulatedArm::sample_sensors() noexcept {
  for (std::size_t i = 0; i < encoders_.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    sample_.joint_positions_rad(row) = encoders_[i].sample(state_.motor_angle(row));
  }

  // What the environment pushes on the tool, re-expressed in the tool frame.
  const Eigen::Matrix3d world_to_tool =
      kinematics::forward_kinematics(model_.arm(), state_.q).linear().transpose();
  math::Wrench in_tool_frame;
  in_tool_frame.head<3>() = world_to_tool * external_wrench_.head<3>();
  in_tool_frame.tail<3>() = world_to_tool * external_wrench_.tail<3>();
  sample_.tool_wrench = force_sensor_.sample(in_tool_frame, control_period_s_);
}

void SimulatedArm::add_to_hash(base::StateHash& hash) const noexcept {
  const auto add_vector = [&hash](const auto& vector) {
    hash.add(std::span<const double>(vector.data(), static_cast<std::size_t>(vector.size())));
  };
  add_vector(state_.q);
  add_vector(state_.qd);
  add_vector(state_.motor_angle);
  add_vector(state_.motor_velocity);
  add_vector(sample_.joint_positions_rad);
  add_vector(sample_.tool_wrench);
}

}  // namespace stapes::plant
