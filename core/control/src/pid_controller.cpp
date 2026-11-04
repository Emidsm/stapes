#include "stapes/control/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace stapes::control {

PidController::PidController(const PidGains& gains, double period_s)
    : gains_(gains),
      period_s_(period_s),
      filter_blend_(gains.derivative_filter_s / (gains.derivative_filter_s + period_s)) {
  if (!(gains.kp >= 0.0) || !(gains.ki >= 0.0) || !(gains.kd >= 0.0) ||
      !(gains.derivative_filter_s >= 0.0)) {
    throw std::invalid_argument("PID gains must not be negative");
  }
  if (!(gains.output_limit > 0.0) || !(period_s > 0.0)) {
    throw std::invalid_argument("PID output limit and period must be positive");
  }
}

double PidController::update(double setpoint, double measurement, double feedforward) noexcept {
  const double limit = gains_.output_limit;
  if (std::isnan(setpoint) || std::isnan(measurement) || std::isnan(feedforward)) {
    return 0.0;
  }

  if (has_last_measurement_) {
    const double rate = (measurement - last_measurement_) / period_s_;
    filtered_rate_ = (filter_blend_ * filtered_rate_) + ((1.0 - filter_blend_) * rate);
  }
  last_measurement_ = measurement;
  has_last_measurement_ = true;

  const double error = setpoint - measurement;
  const double candidate_integral = integral_ + (gains_.ki * error * period_s_);
  const double without_integral = (gains_.kp * error) - (gains_.kd * filtered_rate_) + feedforward;
  const double unclamped = without_integral + candidate_integral;

  const bool pushing_past_limit =
      (unclamped > limit && error > 0.0) || (unclamped < -limit && error < 0.0);
  if (!pushing_past_limit) {
    integral_ = candidate_integral;
  }
  return std::clamp(without_integral + integral_, -limit, limit);
}

void PidController::reset() noexcept {
  integral_ = 0.0;
  filtered_rate_ = 0.0;
  has_last_measurement_ = false;
}

}  // namespace stapes::control
