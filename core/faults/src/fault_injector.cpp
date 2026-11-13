#include "stapes/faults/fault_injector.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace stapes::faults {

FaultInjector::FaultInjector(hardware::RobotInterface& robot, hardware::SafetyInterface& safety,
                             const Fault& fault)
    : robot_(&robot), safety_(&safety), fault_(fault) {
  if (fault.joint >= kinematics::kJointCount) {
    throw std::invalid_argument("fault joint is out of range");
  }
  if (fault.delay_ticks < 0 || fault.delay_ticks > kMaxDelayTicks) {
    throw std::invalid_argument("comms delay is out of range");
  }
  command_history_.fill(kinematics::JointVector::Zero());
  control_sample_ = robot_->read_sensors();
  safety_sample_ = safety_->read_safety_sensors();
  reading_history_.fill(control_sample_);
  now_ticks_ = control_sample_.stamp_ticks;
}

std::size_t FaultInjector::slot(std::int64_t tick) noexcept {
  return static_cast<std::size_t>(tick) % kBufferSize;
}

bool FaultInjector::is_active(FaultKind kind) const noexcept {
  return fault_.kind == kind && active();
}

void FaultInjector::update() noexcept {
  const hardware::SensorSample control = robot_->read_sensors();
  hardware::SafetySample safety = safety_->read_safety_sensors();
  now_ticks_ = control.stamp_ticks;
  reading_history_[slot(now_ticks_)] = control;

  const auto joint = static_cast<Eigen::Index>(fault_.joint);
  hardware::SensorSample delivered = control;
  if (is_active(FaultKind::comms_delay)) {
    const std::int64_t sent = std::max<std::int64_t>(now_ticks_ - fault_.delay_ticks, 0);
    delivered = reading_history_[slot(sent)];
  }
  if (is_active(FaultKind::frozen_motor_encoder)) {
    delivered.joint_positions_rad(joint) = control_sample_.joint_positions_rad(joint);
  }
  if (is_active(FaultKind::frozen_link_encoder)) {
    safety.link_positions_rad(joint) = safety_sample_.link_positions_rad(joint);
  }
  if (is_active(FaultKind::frozen_tracker)) {
    safety.tracked_tool_pose = safety_sample_.tracked_tool_pose;
  }
  control_sample_ = std::move(delivered);
  safety_sample_ = std::move(safety);
}

void FaultInjector::command_torques(const kinematics::JointVector& torques_nm) {
  kinematics::JointVector delivered = torques_nm;
  command_history_[slot(now_ticks_)] = torques_nm;
  if (is_active(FaultKind::comms_delay)) {
    const std::int64_t sent = std::max<std::int64_t>(now_ticks_ - fault_.delay_ticks, 0);
    delivered = command_history_[slot(sent)];
  }
  if (is_active(FaultKind::stuck_actuator)) {
    const auto joint = static_cast<Eigen::Index>(fault_.joint);
    delivered(joint) = last_torques_(joint);
  }
  last_torques_ = delivered;
  robot_->command_torques(delivered);
}

}  // namespace stapes::faults
