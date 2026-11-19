#pragma once

#include "stapes/kinematics/arm_model.hpp"

namespace stapes::task {

/// A move in joint space from one configuration to another, every joint along a straight line
/// and all arriving together.
///
/// The time scaling is the quintic 10s^3 - 15s^4 + 6s^5: velocity and acceleration are zero at
/// both ends, so the move starts from rest and ends at rest with no jump in torque. Its peak
/// speed is 1.875 times the average, at the midpoint.
class JointMove {
 public:
  /// @throws std::invalid_argument if either end is not finite, or the duration is not positive
  ///         and finite.
  JointMove(const kinematics::JointVector& from, const kinematics::JointVector& to,
            double duration_s);

  /// The configuration at `time_s` after the start, held at the ends outside the move.
  [[nodiscard]] kinematics::JointVector at(double time_s) const noexcept;

  /// The fastest each joint moves, in rad/s, reached halfway through the move.
  [[nodiscard]] kinematics::JointVector peak_speeds_rad_s() const noexcept;

  [[nodiscard]] const kinematics::JointVector& from() const noexcept { return from_; }
  [[nodiscard]] const kinematics::JointVector& to() const noexcept { return to_; }
  [[nodiscard]] double duration_s() const noexcept { return duration_s_; }

 private:
  kinematics::JointVector from_;
  kinematics::JointVector to_;
  double duration_s_;
};

}  // namespace stapes::task
