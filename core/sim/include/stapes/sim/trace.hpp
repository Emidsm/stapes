#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "stapes/kinematics/arm_model.hpp"
#include "stapes/math/rotation.hpp"
#include "stapes/safety/safety_monitor.hpp"
#include "stapes/sim/keyhole_simulation.hpp"

namespace stapes::sim {

/// What was true at the end of one control period. This is the simulator's own knowledge: the
/// real tool pose, not a measurement of it. Nothing in the running system reads a trace.
struct TraceSample {
  std::int64_t tick{0};
  double planned_depth_m{0.0};
  kinematics::JointVector q{kinematics::JointVector::Zero()};       ///< True link angles.
  kinematics::JointVector target{kinematics::JointVector::Zero()};  ///< Commanded link angles.
  math::Pose tool{math::Pose::Identity()};                          ///< True tool pose.
  double radial_m{0.0};       ///< True tip to the planned line.
  double axis_offset_m{0.0};  ///< True tool axis to the entry point.
  double depth_m{0.0};        ///< True tip along the tunnel.
  safety::StopCause cause{safety::StopCause::none};
  bool fault_active{false};
  bool brakes_engaged{false};
  bool approaching{false};  ///< Still on the way to the line.
};

/// Records a run for analysis and replay, without disturbing it.
///
/// All storage is reserved in the constructor, so `record` can be called from the tick loop
/// without breaking the rule that a tick does not allocate. A recorder that is full does not
/// grow and does not overwrite: it counts what it had to drop, so a truncated trace can never
/// be mistaken for a complete one.
class TraceRecorder {
 public:
  /// @throws std::invalid_argument if `capacity` is zero.
  explicit TraceRecorder(std::size_t capacity);

  /// Stores one sample of `sim` as it is now, or counts it as dropped if there is no room.
  void record(const KeyholeSimulation& sim) noexcept;

  /// The samples in the order they were recorded.
  [[nodiscard]] std::span<const TraceSample> samples() const noexcept { return samples_; }
  [[nodiscard]] std::size_t capacity() const noexcept { return samples_.capacity(); }
  [[nodiscard]] std::size_t dropped() const noexcept { return dropped_; }

  /// Forgets every sample and the dropped count. Keeps the storage.
  void clear() noexcept;

 private:
  std::vector<TraceSample> samples_;
  std::size_t dropped_{0};
};

/// One sample of `sim` as it is now.
[[nodiscard]] TraceSample sample_of(const KeyholeSimulation& sim) noexcept;

}  // namespace stapes::sim
