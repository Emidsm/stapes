// Python bindings for the simulation. This file translates types and nothing else: every
// function here forwards to one in the core.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>

#include "stapes/base/state_hash.hpp"
#include "stapes/faults/fault_injector.hpp"
#include "stapes/kinematics/arm_model.hpp"
#include "stapes/kinematics/forward_kinematics.hpp"
#include "stapes/safety/safety_monitor.hpp"
#include "stapes/sim/jog_simulation.hpp"
#include "stapes/sim/keyhole_simulation.hpp"
#include "stapes/sim/trace.hpp"
#include "stapes/task/keyhole.hpp"

namespace py = pybind11;

namespace {

using stapes::faults::Fault;
using stapes::faults::FaultKind;
using stapes::kinematics::ArmModel;
using stapes::kinematics::JointVector;
using stapes::kinematics::kJointCount;
using stapes::safety::SafetyConfig;
using stapes::safety::StopCause;
using stapes::sim::AlignRefusal;
using stapes::sim::Approach;
using stapes::sim::JogMode;
using stapes::sim::JogSimulation;
using stapes::sim::KeyholeScenario;
using stapes::sim::KeyholeSimulation;
using stapes::sim::OperatorInput;
using stapes::sim::TraceRecorder;
using stapes::sim::TraceSample;
using stapes::task::KeyholeTrajectory;

using Column = Eigen::VectorXd;
using Table = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// One number per sample, as a NumPy array.
template <typename Field>
Column column(const TraceRecorder& recorder, Field field) {
  const std::span<const TraceSample> samples = recorder.samples();
  Column values(static_cast<Eigen::Index>(samples.size()));
  Eigen::Index row = 0;
  for (const TraceSample& sample : samples) {
    values(row++) = static_cast<double>(field(sample));
  }
  return values;
}

// One fixed-size vector per sample, as a NumPy array with one row per sample.
template <typename Field>
Table table(const TraceRecorder& recorder, Eigen::Index width, Field field) {
  const std::span<const TraceSample> samples = recorder.samples();
  Table values(static_cast<Eigen::Index>(samples.size()), width);
  Eigen::Index row = 0;
  for (const TraceSample& sample : samples) {
    values.row(row++) = field(sample).transpose();
  }
  return values;
}

// The origin of every joint frame followed by the tool tip: the points a viewer joins up.
Eigen::Matrix<double, kJointCount + 1, 3> chain_positions(const ArmModel& model,
                                                          const JointVector& q) {
  const stapes::kinematics::ChainFrames frames = stapes::kinematics::chain_frames(model, q);
  Eigen::Matrix<double, kJointCount + 1, 3> positions;
  Eigen::Index row = 0;
  for (const stapes::math::Pose& frame : frames) {
    positions.row(row++) = frame.translation().transpose();
  }
  return positions;
}

void bind_enums(py::module_& m) {
  py::enum_<FaultKind>(m, "FaultKind")
      .value("none", FaultKind::none)
      .value("frozen_motor_encoder", FaultKind::frozen_motor_encoder)
      .value("frozen_link_encoder", FaultKind::frozen_link_encoder)
      .value("frozen_tracker", FaultKind::frozen_tracker)
      .value("stuck_actuator", FaultKind::stuck_actuator)
      .value("comms_delay", FaultKind::comms_delay);

  py::enum_<StopCause>(m, "StopCause")
      .value("none", StopCause::none)
      .value("non_finite_input", StopCause::non_finite_input)
      .value("stale_data", StopCause::stale_data)
      .value("joint_limit", StopCause::joint_limit)
      .value("joint_velocity", StopCause::joint_velocity)
      .value("tool_force", StopCause::tool_force)
      .value("corridor", StopCause::corridor)
      .value("tracker_disagreement", StopCause::tracker_disagreement)
      .value("encoder_disagreement", StopCause::encoder_disagreement)
      .value("tracking_error", StopCause::tracking_error);
}

void bind_scenario(py::module_& m) {
  py::class_<Fault>(m, "Fault")
      .def(py::init([](FaultKind kind, std::int64_t start_tick, std::size_t joint,
                       int delay_ticks) { return Fault{kind, start_tick, joint, delay_ticks}; }),
           py::arg("kind") = FaultKind::none, py::arg("start_tick") = 0, py::arg("joint") = 0,
           py::arg("delay_ticks") = 0)
      .def_readwrite("kind", &Fault::kind)
      .def_readwrite("start_tick", &Fault::start_tick)
      .def_readwrite("joint", &Fault::joint)
      .def_readwrite("delay_ticks", &Fault::delay_ticks);

  py::class_<ArmModel>(m, "ArmModel")
      .def_static("reference", &ArmModel::reference)
      .def("within_limits", &ArmModel::within_limits);

  py::class_<KeyholeTrajectory>(m, "KeyholeTrajectory")
      .def_property_readonly("entry", &KeyholeTrajectory::entry)
      .def_property_readonly("target", &KeyholeTrajectory::target)
      .def_property_readonly("direction", &KeyholeTrajectory::direction)
      .def_property_readonly("length_m", &KeyholeTrajectory::length_m)
      .def("depth_of", &KeyholeTrajectory::depth_of)
      .def("radial_distance", &KeyholeTrajectory::radial_distance);

  // Read-only: limits are changed in C++, where the monitor's tests cover them.
  py::class_<SafetyConfig>(m, "SafetyConfig")
      .def_property_readonly("corridor_radius_m",
                             [](const SafetyConfig& c) { return c.corridor.radius_m; })
      .def_property_readonly("overshoot_margin_m",
                             [](const SafetyConfig& c) { return c.corridor.overshoot_margin_m; })
      .def_property_readonly("entry_guard_m",
                             [](const SafetyConfig& c) { return c.corridor.entry_guard_m; })
      .def_readonly("period_s", &SafetyConfig::period_s)
      .def_readonly("max_tool_force_n", &SafetyConfig::max_tool_force_n)
      .def_readonly("max_tracker_disagreement_m", &SafetyConfig::max_tracker_disagreement_m)
      .def_readonly("max_sample_age_ticks", &SafetyConfig::max_sample_age_ticks)
      .def_readonly("debounce_checks", &SafetyConfig::debounce_checks);

  py::class_<Approach>(m, "Approach")
      .def_readonly("rest_q", &Approach::rest_q)
      .def_readonly("move_s", &Approach::move_s)
      .def_readonly("settle_s", &Approach::settle_s)
      .def_readonly("standoff_m", &Approach::standoff_m);

  py::class_<KeyholeScenario>(m, "KeyholeScenario")
      .def_static("reference", &KeyholeScenario::reference, py::arg("seed"))
      .def_readwrite("fault", &KeyholeScenario::fault)
      .def_readwrite("feed_speed_m_s", &KeyholeScenario::feed_speed_m_s)
      .def_readwrite("feed_acceleration_m_s2", &KeyholeScenario::feed_acceleration_m_s2)
      .def_readonly("trajectory", &KeyholeScenario::trajectory)
      .def_readonly("safety", &KeyholeScenario::safety)
      .def_readonly("approach", &KeyholeScenario::approach)
      .def_property_readonly("arm", [](const KeyholeScenario& s) { return s.arm.model.arm(); })
      .def_property_readonly("seed", [](const KeyholeScenario& s) { return s.arm.seed; });

  m.def("without_monitor_limits", &stapes::sim::without_monitor_limits, py::arg("scenario"));
  m.def("with_approach", &stapes::sim::with_approach, py::arg("scenario"));
}

void bind_simulation(py::module_& m) {
  py::class_<KeyholeSimulation>(m, "KeyholeSimulation")
      .def(py::init<const KeyholeScenario&>(), py::arg("scenario"))
      .def("step", &KeyholeSimulation::step)
      .def("set_external_wrench", &KeyholeSimulation::set_external_wrench, py::arg("world_wrench"))
      .def_property_readonly("ticks", &KeyholeSimulation::ticks)
      .def_property_readonly("stop_tick", &KeyholeSimulation::stop_tick)
      .def_property_readonly("cause",
                             [](const KeyholeSimulation& sim) { return sim.monitor().cause(); })
      .def_property_readonly("planned_depth_m", &KeyholeSimulation::planned_depth_m)
      .def_property_readonly("feed_duration_s", &KeyholeSimulation::feed_duration_s)
      .def_property_readonly("feed_start_s", &KeyholeSimulation::feed_start_s)
      .def_property_readonly("approaching", &KeyholeSimulation::approaching)
      .def_property_readonly("fault_active", &KeyholeSimulation::fault_active)
      .def_property_readonly(
          "brakes_engaged", [](const KeyholeSimulation& sim) { return sim.arm().brakes_engaged(); })
      .def_property_readonly("target", [](const KeyholeSimulation& sim) { return sim.target(); })
      .def_property_readonly(
          "joint_positions",
          [](const KeyholeSimulation& sim) { return JointVector(sim.arm().state().q); })
      .def("true_tool_pose", [](const KeyholeSimulation& sim) {
        return Eigen::Matrix4d(sim.true_tool_pose().matrix());
      });

  py::class_<stapes::base::StateHash>(m, "StateHash")
      .def(py::init<>())
      .def("add", [](stapes::base::StateHash& hash,
                     const KeyholeSimulation& sim) { sim.add_to_hash(hash); })
      .def_property_readonly("value", &stapes::base::StateHash::value);
}

void bind_trace(py::module_& m) {
  py::class_<TraceRecorder>(m, "TraceRecorder")
      .def(py::init<std::size_t>(), py::arg("capacity"))
      .def("record", &TraceRecorder::record, py::arg("simulation"))
      .def("clear", &TraceRecorder::clear)
      .def("__len__", [](const TraceRecorder& r) { return r.samples().size(); })
      .def_property_readonly("capacity", &TraceRecorder::capacity)
      .def_property_readonly("dropped", &TraceRecorder::dropped)
      .def("tick",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.tick; });
           })
      .def("planned_depth_m",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.planned_depth_m; });
           })
      .def("radial_m",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.radial_m; });
           })
      .def("axis_offset_m",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.axis_offset_m; });
           })
      .def("depth_m",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.depth_m; });
           })
      .def("cause",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return static_cast<int>(s.cause); });
           })
      .def("fault_active",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.fault_active; });
           })
      .def("brakes_engaged",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.brakes_engaged; });
           })
      .def("approaching",
           [](const TraceRecorder& r) {
             return column(r, [](const TraceSample& s) { return s.approaching; });
           })
      .def("q",
           [](const TraceRecorder& r) {
             return table(r, kJointCount, [](const TraceSample& s) { return s.q; });
           })
      .def("target",
           [](const TraceRecorder& r) {
             return table(r, kJointCount, [](const TraceSample& s) { return s.target; });
           })
      .def("tip_m",
           [](const TraceRecorder& r) {
             return table(
                 r, 3, [](const TraceSample& s) { return Eigen::Vector3d(s.tool.translation()); });
           })
      .def("tool_axis", [](const TraceRecorder& r) {
        return table(r, 3, [](const TraceSample& s) { return stapes::task::tool_axis(s.tool); });
      });
}

void bind_jog(py::module_& m) {
  py::enum_<JogMode>(m, "JogMode")
      .value("free", JogMode::free)
      .value("aligning", JogMode::aligning)
      .value("on_line", JogMode::on_line)
      .value("joint", JogMode::joint);

  py::enum_<AlignRefusal>(m, "AlignRefusal")
      .value("none", AlignRefusal::none)
      .value("still_moving", AlignRefusal::still_moving)
      .value("enters_guard", AlignRefusal::enters_guard)
      .value("not_at_standoff", AlignRefusal::not_at_standoff);

  py::class_<OperatorInput>(m, "OperatorInput")
      .def(py::init([](const Eigen::Vector3d& tip_velocity_m_s, double feed, bool align, bool fault,
                       bool joint_mode, int joint, double joint_rate) {
             return OperatorInput{tip_velocity_m_s, feed,  align,     fault,
                                  joint_mode,       joint, joint_rate};
           }),
           py::arg("tip_velocity_m_s") = Eigen::Vector3d::Zero(), py::arg("feed") = 0.0,
           py::arg("align") = false, py::arg("fault") = false, py::arg("joint_mode") = false,
           py::arg("joint") = 0, py::arg("joint_rate") = 0.0)
      .def_readwrite("tip_velocity_m_s", &OperatorInput::tip_velocity_m_s)
      .def_readwrite("feed", &OperatorInput::feed)
      .def_readwrite("align", &OperatorInput::align)
      .def_readwrite("fault", &OperatorInput::fault)
      .def_readwrite("joint_mode", &OperatorInput::joint_mode)
      .def_readwrite("joint", &OperatorInput::joint)
      .def_readwrite("joint_rate", &OperatorInput::joint_rate);

  py::class_<JogSimulation>(m, "JogSimulation")
      .def(py::init<const KeyholeScenario&>(), py::arg("scenario"))
      .def("step", &JogSimulation::step, py::arg("input"))
      .def_readonly_static("MAX_TIP_SPEED_M_S", &JogSimulation::kMaxTipSpeedMS)
      .def_property_readonly("mode", &JogSimulation::mode)
      .def_property_readonly("align_refusal", &JogSimulation::align_refusal)
      .def_property_readonly("ticks", &JogSimulation::ticks)
      .def_property_readonly("planned_depth_m", &JogSimulation::planned_depth_m)
      .def_property_readonly("fault_active", &JogSimulation::fault_active)
      .def_property_readonly("fault_start_tick", &JogSimulation::fault_start_tick)
      .def_property_readonly("stop_tick", &JogSimulation::stop_tick)
      .def_property_readonly("cause",
                             [](const JogSimulation& sim) { return sim.monitor().cause(); })
      .def_property_readonly(
          "joint_positions",
          [](const JogSimulation& sim) { return JointVector(sim.arm().state().q); })
      .def("true_tool_pose",
           [](const JogSimulation& sim) { return Eigen::Matrix4d(sim.true_tool_pose().matrix()); })
      .def("state_hash", [](const JogSimulation& sim) {
        stapes::base::StateHash hash;
        sim.add_to_hash(hash);
        return hash.value();
      });

  m.def("jog_mode_name", [](JogMode mode) { return std::string(stapes::sim::to_string(mode)); });
  m.def("align_refusal_name",
        [](AlignRefusal refusal) { return std::string(stapes::sim::to_string(refusal)); });
}

}  // namespace

PYBIND11_MODULE(stapes_core, m) {
  m.doc() = "Stapes: a deterministic simulation environment for a 6-DOF surgical robot arm.";
  m.attr("__version__") = STAPES_VERSION;
  m.attr("compiler") = std::string(
#if defined(__clang__)
      "Clang "
#else
      "GCC "
#endif
      __VERSION__);
  m.attr("JOINT_COUNT") = kJointCount;

  bind_enums(m);
  bind_scenario(m);
  bind_simulation(m);
  bind_trace(m);
  bind_jog(m);

  m.def("stop_cause_name",
        [](StopCause cause) { return std::string(stapes::safety::to_string(cause)); });
  m.def("reference_home", &stapes::kinematics::reference_home);
  m.def("forward_kinematics", [](const ArmModel& model, const JointVector& q) {
    return Eigen::Matrix4d(stapes::kinematics::forward_kinematics(model, q).matrix());
  });
  m.def("chain_positions", &chain_positions, py::arg("model"), py::arg("q"));
}
