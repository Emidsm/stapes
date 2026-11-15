// Replaces the global allocation functions in this binary to count calls, so this file
// stays in its own test executable.

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>

#include <gtest/gtest.h>

#include "requirement.hpp"
#include "stapes/base/random.hpp"
#include "stapes/control/joint_controller.hpp"
#include "stapes/dynamics/rigid_body_dynamics.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/kinematics/inverse_kinematics.hpp"
#include "stapes/plant/simulated_arm.hpp"
#include "stapes/sim/keyhole_simulation.hpp"
#include "stapes/sim/trace.hpp"

namespace {

std::atomic<long> allocation_count{0};

void* counted_allocate(std::size_t size) {
  allocation_count.fetch_add(1, std::memory_order_relaxed);
  // NOLINTNEXTLINE(*-no-malloc, *-owning-memory)
  void* memory = std::malloc(size == 0 ? 1 : size);
  if (memory == nullptr) {
    throw std::bad_alloc();
  }
  return memory;
}

void* counted_allocate_aligned(std::size_t size, std::align_val_t alignment) {
  allocation_count.fetch_add(1, std::memory_order_relaxed);
  const auto boundary = static_cast<std::size_t>(alignment);
  // std::aligned_alloc requires the size to be a multiple of the alignment.
  const std::size_t rounded = ((size + boundary - 1) / boundary) * boundary;
  // NOLINTNEXTLINE(*-no-malloc, *-owning-memory)
  void* memory = std::aligned_alloc(boundary, rounded == 0 ? boundary : rounded);
  if (memory == nullptr) {
    throw std::bad_alloc();
  }
  return memory;
}

}  // namespace

// NOLINTBEGIN(*-no-malloc, *-owning-memory, misc-new-delete-overloads)
void* operator new(std::size_t size) { return counted_allocate(size); }
void* operator new[](std::size_t size) { return counted_allocate(size); }
void* operator new(std::size_t size, std::align_val_t alignment) {
  return counted_allocate_aligned(size, alignment);
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
  return counted_allocate_aligned(size, alignment);
}
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t /*size*/) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t /*size*/) noexcept { std::free(memory); }
void operator delete(void* memory, std::align_val_t /*alignment*/) noexcept { std::free(memory); }
void operator delete[](void* memory, std::align_val_t /*alignment*/) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t /*size*/, std::align_val_t /*alignment*/) noexcept {
  std::free(memory);
}
void operator delete[](void* memory, std::size_t /*size*/,
                       std::align_val_t /*alignment*/) noexcept {
  std::free(memory);
}
// NOLINTEND(*-no-malloc, *-owning-memory, misc-new-delete-overloads)

namespace {

using stapes::kinematics::ArmModel;
using stapes::kinematics::forward_kinematics;
using stapes::kinematics::geometric_jacobian;
using stapes::kinematics::JointVector;
using stapes::kinematics::manipulability;
using stapes::kinematics::reference_home;
using stapes::kinematics::solve_ik;

template <typename Function>
long allocations_during(Function function) {
  const long before = allocation_count.load(std::memory_order_relaxed);
  function();
  return allocation_count.load(std::memory_order_relaxed) - before;
}

TEST(NoAllocation, TheCounterDetectsAnAllocation) {
  // Negative control: without this, a broken counter would make every other test pass.
  stapes::test::verifies("SRS-RT-001");
  const long counted = allocations_during([] {
    void* memory = ::operator new(16);
    ::operator delete(memory);
  });
  EXPECT_EQ(counted, 1);
}

TEST(NoAllocation, RandomDrawsDoNotAllocate) {
  stapes::test::verifies("SRS-RT-001");
  stapes::base::Rng rng(1);
  double sink = 0.0;
  const long counted = allocations_during([&] {
    for (int i = 0; i < 1000; ++i) {
      sink += rng.uniform01() + rng.gaussian();
    }
  });
  EXPECT_EQ(counted, 0);
  EXPECT_TRUE(std::isfinite(sink));
}

TEST(NoAllocation, ForwardKinematicsAndJacobianDoNotAllocate) {
  stapes::test::verifies("SRS-RT-001");
  const ArmModel model = ArmModel::reference();
  const JointVector q = reference_home();
  double sink = 0.0;
  const long counted = allocations_during([&] {
    sink += forward_kinematics(model, q).translation().x();
    sink += geometric_jacobian(model, q)(0, 0);
    sink += manipulability(model, q);
  });
  EXPECT_EQ(counted, 0);
  EXPECT_TRUE(std::isfinite(sink));
}

TEST(NoAllocation, InverseKinematicsDoesNotAllocate) {
  stapes::test::verifies("SRS-RT-001");
  const ArmModel model = ArmModel::reference();
  JointVector goal = reference_home();
  goal(0) += 0.5;
  goal(2) += 0.3;
  const stapes::math::Pose target = forward_kinematics(model, goal);

  int iterations = 0;
  const long counted = allocations_during(
      [&] { iterations = solve_ik(model, target, reference_home()).iterations; });
  EXPECT_EQ(counted, 0);
  EXPECT_GT(iterations, 0);
}

TEST(NoAllocation, DynamicsDoNotAllocate) {
  stapes::test::verifies("SRS-RT-002");
  const stapes::dynamics::DynamicModel model = stapes::dynamics::DynamicModel::reference();
  const JointVector q = reference_home();
  const JointVector qd = JointVector::Constant(0.5);
  double sink = 0.0;
  const long counted = allocations_during([&] {
    sink += stapes::dynamics::inverse_dynamics(model, q, qd, qd)(0);
    sink += stapes::dynamics::mass_matrix(model, q)(0, 0);
    sink += stapes::dynamics::forward_dynamics(model, q, qd, qd)(0);
    sink += stapes::dynamics::kinetic_energy(model, q, qd);
    sink += stapes::dynamics::potential_energy(model, q);
  });
  EXPECT_EQ(counted, 0);
  EXPECT_TRUE(std::isfinite(sink));
}

TEST(NoAllocation, AClosedLoopTickDoesNotAllocate) {
  stapes::test::verifies("SRS-RT-002");
  const stapes::plant::SimulatedArmConfig config = stapes::plant::SimulatedArmConfig::reference(1);
  stapes::plant::SimulatedArm arm(config);
  stapes::control::JointController controller(config.model, stapes::control::reference_gains(),
                                              stapes::control::reference_gearbox_estimates(), 1e-3);
  const JointVector target = reference_home();
  const long counted = allocations_during([&] {
    for (int i = 0; i < 100; ++i) {
      arm.command_torques(controller.update(target, arm.read_sensors().joint_positions_rad));
      arm.tick();
    }
  });
  EXPECT_EQ(counted, 0);
  EXPECT_EQ(arm.clock().ticks(), 100);
}

TEST(NoAllocation, AFullSimulationTickWithAFaultAndAMonitorDoesNotAllocate) {
  stapes::test::verifies("SRS-RT-003");
  stapes::sim::KeyholeScenario scenario = stapes::sim::KeyholeScenario::reference(1);
  scenario.fault = stapes::faults::Fault{stapes::faults::FaultKind::comms_delay, 10, 0, 1};
  stapes::sim::KeyholeSimulation sim(scenario);
  const long counted = allocations_during([&] {
    for (int i = 0; i < 100; ++i) {
      sim.step();
    }
  });
  EXPECT_EQ(counted, 0);
  EXPECT_EQ(sim.ticks(), 100);
}

TEST(NoAllocation, ASimulationTickAfterAStopDoesNotAllocate) {
  stapes::test::verifies("SRS-RT-003");
  stapes::sim::KeyholeScenario scenario = stapes::sim::KeyholeScenario::reference(1);
  scenario.fault = stapes::faults::Fault{stapes::faults::FaultKind::comms_delay, 0, 0, 100};
  stapes::sim::KeyholeSimulation sim(scenario);
  const long counted = allocations_during([&] {
    for (int i = 0; i < 200; ++i) {
      sim.step();
    }
  });
  EXPECT_EQ(counted, 0);
}

TEST(NoAllocation, RecordingATraceDoesNotAllocateEvenWhenFull) {
  stapes::test::verifies("SRS-TRC-002");
  stapes::sim::KeyholeSimulation sim(stapes::sim::KeyholeScenario::reference(1));
  stapes::sim::TraceRecorder recorder(50);
  const long counted = allocations_during([&] {
    for (int i = 0; i < 80; ++i) {
      sim.step();
      recorder.record(sim);
    }
  });
  EXPECT_EQ(counted, 0);
  EXPECT_EQ(recorder.samples().size(), 50U);
  EXPECT_EQ(recorder.dropped(), 30U);
}

TEST(NoAllocation, MultistartInverseKinematicsDoesNotAllocate) {
  stapes::test::verifies("SRS-RT-001");
  const ArmModel model = ArmModel::reference();
  // Out of reach, so every start runs and the restart path is the one measured.
  stapes::math::Pose target = stapes::math::Pose::Identity();
  target.translation() = Eigen::Vector3d(2.0, 0.0, 0.3);
  stapes::base::Rng rng(2);

  int iterations = 0;
  const long counted = allocations_during([&] {
    iterations =
        stapes::kinematics::solve_ik_multistart(model, target, reference_home(), rng, 4).iterations;
  });
  EXPECT_EQ(counted, 0);
  EXPECT_GT(iterations, 0);
}

}  // namespace
