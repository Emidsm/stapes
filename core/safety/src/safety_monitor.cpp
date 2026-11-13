// STUB. The owner writes this file. tests/unit/safety/test_safety_monitor.cpp fails until then.
#include "stapes/safety/safety_monitor.hpp"

namespace stapes::safety {

SafetyMonitor::SafetyMonitor(const SafetyConfig& /*config*/) {}

StopCause SafetyMonitor::check(const SafetyInput& /*input*/) noexcept { return cause_; }

bool SafetyMonitor::try_reset(const SafetyInput& /*input*/) noexcept { return true; }

}  // namespace stapes::safety
