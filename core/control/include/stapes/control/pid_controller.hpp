#pragma once

namespace stapes::control {

struct PidGains {
  double kp;
  double ki;
  double kd;
  double output_limit;
  /// Time constant of the low-pass filter on the derivative. Differentiating quantised
  /// encoder counts directly would turn each count into a torque spike.
  double derivative_filter_s;
};

/// A discrete PID controller for one joint.
///
/// The derivative acts on the measurement, not the error, so a step in the setpoint does
/// not produce a torque spike. The integral stops growing while the output is saturated
/// in the direction it would grow, which prevents windup.
class PidController {
 public:
  /// @throws std::invalid_argument if a gain is negative, or the period or limit not positive.
  PidController(const PidGains& gains, double period_s);

  /// Computes the output for one period. `feedforward` is added inside the output limit.
  /// Any NaN input produces zero and leaves the internal state unchanged, so one bad
  /// sample cannot poison the integral for the rest of the run.
  [[nodiscard]] double update(double setpoint, double measurement,
                              double feedforward = 0.0) noexcept;

  /// Clears the integral and derivative history.
  void reset() noexcept;

  [[nodiscard]] double integral() const noexcept { return integral_; }

 private:
  PidGains gains_;
  double period_s_;
  double filter_blend_;
  double integral_{0.0};
  double filtered_rate_{0.0};
  double last_measurement_{0.0};
  bool has_last_measurement_{false};
};

}  // namespace stapes::control
