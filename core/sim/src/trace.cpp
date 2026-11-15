#include "stapes/sim/trace.hpp"

#include <stdexcept>

#include "stapes/task/keyhole.hpp"

namespace stapes::sim {

TraceSample sample_of(const KeyholeSimulation& sim) noexcept {
  const math::Pose tool = sim.true_tool_pose();
  const task::KeyholeTrajectory& tunnel = sim.trajectory();
  TraceSample sample;
  sample.tick = sim.ticks();
  sample.planned_depth_m = sim.planned_depth_m();
  sample.q = sim.arm().state().q;
  sample.target = sim.target();
  sample.tool = tool;
  sample.radial_m = tunnel.radial_distance(tool.translation());
  sample.axis_offset_m = task::axis_distance(tool, tunnel.entry());
  sample.depth_m = tunnel.depth_of(tool.translation());
  sample.cause = sim.monitor().cause();
  sample.fault_active = sim.fault_active();
  sample.brakes_engaged = sim.arm().brakes_engaged();
  return sample;
}

TraceRecorder::TraceRecorder(std::size_t capacity) {
  if (capacity == 0) {
    throw std::invalid_argument("a trace needs room for at least one sample");
  }
  samples_.reserve(capacity);
}

void TraceRecorder::record(const KeyholeSimulation& sim) noexcept {
  if (samples_.size() == samples_.capacity()) {
    ++dropped_;
    return;
  }
  // Within the reserved capacity, so this constructs in place and cannot allocate.
  samples_.push_back(sample_of(sim));
}

void TraceRecorder::clear() noexcept {
  samples_.clear();
  dropped_ = 0;
}

}  // namespace stapes::sim
