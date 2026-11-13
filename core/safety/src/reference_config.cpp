#include "stapes/safety/safety_monitor.hpp"

namespace stapes::safety {

const char* to_string(StopCause cause) noexcept {
  switch (cause) {
    case StopCause::none:
      return "none";
    case StopCause::non_finite_input:
      return "non_finite_input";
    case StopCause::stale_data:
      return "stale_data";
    case StopCause::joint_limit:
      return "joint_limit";
    case StopCause::joint_velocity:
      return "joint_velocity";
    case StopCause::tool_force:
      return "tool_force";
    case StopCause::corridor:
      return "corridor";
    case StopCause::tracker_disagreement:
      return "tracker_disagreement";
    case StopCause::encoder_disagreement:
      return "encoder_disagreement";
    case StopCause::tracking_error:
      return "tracking_error";
  }
  return "unknown";
}

SafetyConfig reference_safety_config(const Corridor& corridor) {
  kinematics::JointVector max_velocity;
  max_velocity.setConstant(0.2);
  kinematics::JointVector max_encoder_disagreement;
  max_encoder_disagreement.setConstant(1.0e-3);
  kinematics::JointVector max_tracking_error;
  max_tracking_error.setConstant(1.8e-3);
  return SafetyConfig{
      kinematics::ArmModel::reference(), corridor,           0.02, 1e-3, max_velocity, 20.0, 0.3e-3,
      max_encoder_disagreement,          max_tracking_error, 2,    3,
  };
}

}  // namespace stapes::safety
