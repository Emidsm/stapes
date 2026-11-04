#include "stapes/base/sim_clock.hpp"

#include <stdexcept>

namespace stapes::base {

SimClock::SimClock(SimDuration step) : step_(step) {
  if (step <= SimDuration::zero()) {
    throw std::invalid_argument("SimClock step must be positive");
  }
}

}  // namespace stapes::base
