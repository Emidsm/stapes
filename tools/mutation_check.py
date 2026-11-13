#!/usr/bin/env python3
"""Inject known bugs, one at a time, and check that the test suite catches each one.

Line coverage only shows that a line ran. This shows that a test would notice if the
line were wrong. It works on a temporary copy and never touches the working tree.

Exits non-zero if a bug survives or a pattern no longer matches the code, so the list
cannot silently go stale.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import NamedTuple


class Mutation(NamedTuple):
    name: str
    path: str
    original: str
    replacement: str


FK = "core/kinematics/src/forward_kinematics.cpp"
MODEL = "core/kinematics/src/arm_model.cpp"
IK = "core/kinematics/src/inverse_kinematics.cpp"
RANDOM = "core/base/src/random.cpp"
CLOCK = "core/base/src/sim_clock.cpp"
ROTATION = "core/math/src/rotation.cpp"
MODEL_DYN = "core/dynamics/src/dynamic_model.cpp"
DYN = "core/dynamics/src/rigid_body_dynamics.cpp"
ACT = "core/plant/src/actuator.cpp"
ENC = "core/plant/src/encoder.cpp"
FORCE = "core/plant/src/force_sensor.cpp"
ARM = "core/plant/src/simulated_arm.cpp"
PID = "core/control/src/pid_controller.cpp"
JOINT = "core/control/src/joint_controller.cpp"
TRACKER = "core/plant/src/optical_tracker.cpp"
KEYHOLE = "core/task/src/keyhole.cpp"
INJECTOR = "core/faults/src/fault_injector.cpp"
SIM = "core/sim/src/keyhole_simulation.cpp"
STREAM_MIX = "seed_mixer.next() ^ std::rotl(stream_mixer.next(), 32)"

MUTATIONS = [
    Mutation("Jacobian lever arm reversed", FK, "axis_world.cross(lever_arm)",
             "lever_arm.cross(axis_world)"),
    Mutation("tool transform ignored", FK, "frames.back() = current * model.tool();",
             "frames.back() = current;"),
    Mutation("upper limit check excludes the limit", MODEL, "angle <= joints_[i].upper_rad",
             "angle < joints_[i].upper_rad"),
    Mutation("reference arm link length wrong", MODEL, "revolute({0.0, 0.0, 0.30}, y",
             "revolute({0.0, 0.0, 0.31}, y"),
    Mutation("xoshiro rotation wrong", RANDOM, "std::rotl(state_[0] + state_[3], 23)",
             "std::rotl(state_[0] + state_[3], 24)"),
    Mutation("Gaussian scale wrong", RANDOM, "std::sqrt(-2.0 * std::log",
             "std::sqrt(-1.9 * std::log"),
    Mutation("Gaussian second sample discarded", RANDOM, "has_spare_gaussian_ = true;",
             "has_spare_gaussian_ = false;"),
    Mutation("uniform01 scale wrong", RANDOM, "next_u64() >> 11U", "next_u64() >> 12U"),
    Mutation("stream ignored when seeding", RANDOM, STREAM_MIX, "seed_mixer.next()"),
    Mutation("stream and seed combined symmetrically", RANDOM, STREAM_MIX,
             "seed_mixer.next() ^ stream_mixer.next()"),
    Mutation("clock accepts a zero step", CLOCK, "step <= SimDuration::zero()",
             "step < SimDuration::zero()"),
    Mutation("quaternion sign not normalised", ROTATION, "if (quaternion.w() < 0.0) {",
             "if (false) {"),
    Mutation("small-angle branch wrong", ROTATION, "return 2.0 * quaternion.vec();",
             "return quaternion.vec();"),
    Mutation("orientation error reversed", ROTATION,
             "rotation_vector(target * actual.transpose())",
             "rotation_vector(actual * target.transpose())"),
    Mutation("IK accepts either tolerance", IK, "options.position_tolerance_m &&",
             "options.position_tolerance_m ||"),
    Mutation("IK does not clamp candidates", IK, "model.clamp_to_limits(q + step)",
             "JointVector(q + step)"),
    Mutation("IK does not clamp the seed", IK, "JointVector q = model.clamp_to_limits(seed);",
             "JointVector q = seed;"),
    Mutation("IK does not freeze joints at limits", IK, "if (pushed_above || pushed_below) {",
             "if (false) {"),
    Mutation("IK step limit disabled", IK, "largest > max_step_rad ?", "false ?"),
    Mutation("IK accepts non-finite input", IK,
             "if (!seed.allFinite() || !target.matrix().allFinite()) {", "if (false) {"),
    Mutation("IK claims success when out of iterations", IK,
             "within_tolerance(current, options) ? IkStatus::converged : IkStatus::max_iterations",
             "IkStatus::converged"),
    Mutation("IK negligible-improvement stop removed", IK,
             "(negligible && !within_tolerance(current, options))", "false"),
    Mutation("multistart keeps the worst attempt", IK,
             "weighted_error(attempt) < weighted_error(best)",
             "weighted_error(attempt) > weighted_error(best)"),
    Mutation("multistart continues after success", IK,
             "start < max_starts && !best.converged() &&", "start < max_starts &&"),
    Mutation("multistart restarts on invalid input", IK,
             "best.status != IkStatus::invalid_input", "true"),
    Mutation("multistart under-reports iterations", IK, "best.iterations = total_iterations;", ""),
    Mutation("impossible inertia accepted", MODEL_DYN, "moments(0) + moments(1) >= moments(2)",
             "true"),
    Mutation("gravity sign reversed", DYN, "accel = -model.gravity();",
             "accel = model.gravity();"),
    Mutation("Coriolis term dropped", DYN, "carried_omega.cross(axis * qd(row))",
             "Eigen::Vector3d::Zero()"),
    Mutation("gyroscopic term dropped", DYN, " + omega.cross(link.inertia_kgm2 * omega)", ""),
    Mutation("centre-of-mass moment dropped", DYN, "links[i].com_m.cross(forces[i]) + ", ""),
    Mutation("composite inertia not accumulated", DYN,
             "composite[i - 1] += transforms[i].transpose() * composite[i] * transforms[i];", ""),
    Mutation("spatial transform sign wrong", DYN, "-parent_to_link * skew(offset)",
             "parent_to_link * skew(offset)"),
    Mutation("potential energy sign wrong", DYN, "energy -= model.links()",
             "energy += model.links()"),
    Mutation("friction not smoothed across zero", ACT,
             "std::tanh(velocity / params.smoothing_velocity_rad_s)", "1.0"),
    Mutation("Stribeck peak does not decay", ACT, "std::exp(-normalised * normalised)", "1.0"),
    Mutation("viscous friction dropped", ACT, "(params.viscous_nm_s * velocity)", "0.0"),
    Mutation("backlash ignored on the positive side", ACT,
             "deflection > params.backlash_half_width_rad", "deflection > 0.0"),
    Mutation("gear teeth can pull, positive side", ACT, "std::max(push, 0.0)", "push"),
    Mutation("gear teeth can pull, negative side", ACT, "std::min(push, 0.0)", "push"),
    Mutation("NaN torque command passed through", ACT, "std::isnan(command) ? 0.0 : ", ""),
    Mutation("encoder truncates rather than rounds", ENC, "std::round(noisy", "std::floor(noisy"),
    Mutation("encoder ignores noise", ENC, "params_.noise_stddev_rad * rng_.gaussian()", "0.0"),
    Mutation("force drift grows with time, not its root", FORCE, "drift * root_elapsed",
             "drift * elapsed_s"),
    Mutation("force scale error ignored", FORCE, "(1.0 + params_.scale_error) * true_wrench",
             "true_wrench"),
    Mutation("torque channels use force noise", FORCE, "channel < 3", "channel < 6"),
    Mutation("encoders read the link", ARM, "encoders_[i].sample(state_.motor_angle(row))",
             "encoders_[i].sample(state_.q(row))"),
    Mutation("all encoders share one noise stream", ARM, "stream_base + Index",
             "stream_base + (Index * 0)"),
    Mutation("force reported in the world frame", ARM,
             "world_to_tool * external_wrench_.head<3>()", "external_wrench_.head<3>()"),
    Mutation("external wrench does not act on the arm", ARM, "gear + environment", "gear"),
    Mutation("motor friction not applied", ARM, " - friction - gear(row)", " - gear(row)"),
    Mutation("link positions never integrated", ARM, "state_.q += state_.qd * step_s;", ""),
    Mutation("PID error sign reversed", PID, "setpoint - measurement", "measurement - setpoint"),
    Mutation("PID anti-windup disabled", PID, "if (!pushing_past_limit) {", "if (true) {"),
    Mutation("PID cannot unwind while saturated", PID, "unclamped > limit && error > 0.0",
             "unclamped > limit"),
    Mutation("PID output not clamped", PID, "std::clamp(without_integral + integral_, -limit, limit)",
             "without_integral + integral_"),
    Mutation("PID accepts NaN feedforward", PID, " || std::isnan(feedforward)", ""),
    Mutation("PID reset keeps the filtered rate", PID, "filtered_rate_ = 0.0;", ""),
    Mutation("PID reset keeps the last measurement", PID, "has_last_measurement_ = false;", ""),
    Mutation("controller ignores gearbox deflection", JOINT,
             "(load(row) / gearboxes_[i].stiffness_nm_per_rad)", "0.0"),
    Mutation("controller ignores backlash", JOINT, "std::tanh(load(row) / kBacklashBlendTorque)",
             "0.0"),
    Mutation("controller drops gravity compensation", JOINT,
             "pids_[i].update(motor(row), measured(row), gravity(row))",
             "pids_[i].update(motor(row), measured(row))"),
    Mutation("controller aims at the link, not the motor", JOINT,
             "pids_[i].update(motor(row), measured(row), gravity(row))",
             "pids_[i].update(target(row), measured(row), gravity(row))"),
    Mutation("tracker ignores position noise", TRACKER,
             "params_.position_noise_m * rng_.gaussian()", "0.0"),
    Mutation("tracker tilts without keeping the original rotation", TRACKER,
             "true_pose.linear() * Eigen::AngleAxisd(angle, tilt / angle)",
             "Eigen::AngleAxisd(angle, tilt / angle).toRotationMatrix()"),
    Mutation("tracker accepts negative noise", TRACKER, "params.position_noise_m < 0.0",
             "params.position_noise_m < -1e9"),
    Mutation("gearbox starts unloaded", ARM, "angles(row) += (load(row) / gear.stiffness_nm_per_rad)",
             "angles(row) += 0.0 * (load(row) / gear.stiffness_nm_per_rad)"),
    Mutation("gearbox starts on the wrong tooth face", ARM,
             "std::copysign(gear.backlash_half_width_rad, load(row))",
             "std::copysign(gear.backlash_half_width_rad, -load(row))"),
    Mutation("link encoders read the motor", ARM, "link_encoders_[i].sample(state_.q(row))",
             "link_encoders_[i].sample(state_.motor_angle(row))"),
    Mutation("commands still accepted with brakes on", ARM, """if (brakes_engaged_) {
    return;
  }""", ""),
    Mutation("brake leaves torque applied", ARM, "applied_torques_.setZero();", ""),
    Mutation("brake not limited to its rating", ARM,
             "std::clamp(to_stop, -brake_torque_nm_(row), brake_torque_nm_(row))", "to_stop"),
    Mutation("brake ignores the motor's speed", ARM,
             "(actuator.rotor_inertia_kgm2 * state_.motor_velocity(row) / step_s)", "0.0"),
    Mutation("safety sample not stamped", ARM, "safety_sample_.stamp_ticks = clock_.ticks();", ""),
    Mutation("control sample not stamped", ARM, "  sample_.stamp_ticks = clock_.ticks();", ""),
    Mutation("brake state missing from the hash", ARM,
             "hash.add(static_cast<std::uint64_t>(brakes_engaged_));", ""),
    Mutation("axis distance measured from the tip", KEYHOLE,
             "tool_axis(tool).cross(point - tool.translation()).norm()",
             "(point - tool.translation()).norm()"),
    Mutation("tool axis is the wrong column", KEYHOLE, "tool.linear().col(0)",
             "tool.linear().col(2)"),
    Mutation("pose keeps the reference rotation", KEYHOLE,
             "turn.toRotationMatrix() * reference.linear()", "reference.linear()"),
    Mutation("fulcrum axis points the wrong way", KEYHOLE, "(tip - fulcrum).normalized()",
             "(fulcrum - tip).normalized()"),
    Mutation("tunnel depth not clamped", KEYHOLE, "std::clamp(depth_m, 0.0, length_m_)",
             "depth_m"),
    Mutation("radial distance includes depth", KEYHOLE, "direction_.cross(point - entry_).norm()",
             "(point - entry_).norm()"),
    Mutation("short feed keeps the speed limit", KEYHOLE,
             "std::min(speed_m_s, std::sqrt(length_m * acceleration_m_s2))", "speed_m_s"),
    Mutation("feed duration misses a ramp", KEYHOLE, "(length_m / peak_speed_m_s_) + ramp_s_",
             "(length_m / peak_speed_m_s_)"),
    Mutation("plan check stops after the first point", KEYHOLE,
             "while (depth < trajectory.length_m()) {", "while (false) {"),
    Mutation("plan check ignores a failure part way", KEYHOLE, """if (!next.converged()) {
      return false;
    }""", ""),
    Mutation("plan check accepts a bad start", KEYHOLE, """if (!first.converged()) {
    return false;
  }""", ""),
    Mutation("frozen encoder reports the live value", INJECTOR,
             "delivered.joint_positions_rad(joint) = control_sample_.joint_positions_rad(joint);",
             ""),
    Mutation("frozen link encoder reports the live value", INJECTOR,
             "safety.link_positions_rad(joint) = safety_sample_.link_positions_rad(joint);", ""),
    Mutation("frozen tracker reports the live pose", INJECTOR,
             "safety.tracked_tool_pose = safety_sample_.tracked_tool_pose;", ""),
    Mutation("stuck actuator accepts commands", INJECTOR,
             "delivered(joint) = last_torques_(joint);", ""),
    Mutation("readings are not delayed", INJECTOR,
             "delivered = reading_history_[slot(sent)];", ""),
    Mutation("commands are not delayed", INJECTOR, "delivered = command_history_[slot(sent)];", ""),
    Mutation("fault active before its start", INJECTOR, "now_ticks_ = control.stamp_ticks;",
             "now_ticks_ = control.stamp_ticks + 1;"),
    Mutation("delay buffer index wrong after wrap-around", INJECTOR,
             "static_cast<std::size_t>(tick) % kBufferSize",
             "static_cast<std::size_t>(tick) % (kBufferSize - 1)"),
    Mutation("controller keeps running after a stop", SIM, """    injector_.engage_brakes();
    stop_tick_ = arm_.clock().ticks();
  } else {
    injector_.command_torques""", """    stop_tick_ = arm_.clock().ticks();
  } else {
    injector_.command_torques"""),
    Mutation("arm does not start on the plan", SIM, "config.initial_q = start;", ""),
    Mutation("the monitor is never consulted", SIM,
             "monitor_.check(input) != safety::StopCause::none", "false"),
    Mutation("reachability is not checked", SIM, """throw std::invalid_argument("the planned tunnel is not reachable");""", ""),
]

TEST_TIMEOUT_S = 300


def run(command: list[str], cwd: Path) -> subprocess.CompletedProcess:
    return subprocess.run(command, cwd=cwd, capture_output=True, text=True, check=False)


def tests_catch_it(work: Path) -> bool:
    try:
        result = subprocess.run(["ctest", "--test-dir", "build", "--timeout", "120", "--parallel",
                                 str(os.cpu_count() or 1), "--stop-on-failure"], cwd=work,
                                capture_output=True, check=False, timeout=TEST_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return True  # A hang is a detected failure.
    return result.returncode != 0


def prepare(root: Path, deps: Path, work: Path) -> None:
    shutil.copytree(root, work, ignore=shutil.ignore_patterns("build", ".git"))
    steps = [
        ["cmake", "-S", ".", "-B", "build", "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Debug",
         "-DCMAKE_CXX_FLAGS_DEBUG=-g -O1", "-DSTAPES_BUILD_BENCHMARKS=OFF",
         f"-DFETCHCONTENT_SOURCE_DIR_EIGEN={deps}/eigen-src",
         f"-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST={deps}/googletest-src"],
        ["cmake", "--build", "build"],
        ["ctest", "--test-dir", "build"],
    ]
    for step in steps:
        result = run(step, work)
        if result.returncode != 0:
            raise RuntimeError(f"baseline failed: {' '.join(step)}\n{result.stdout}{result.stderr}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deps", type=Path, default=Path("build/debug/_deps"),
                        help="existing FetchContent directory, to avoid downloading again")
    parser.add_argument("--match", default="",
                        help="only run mutations whose name contains this text")
    args = parser.parse_args()
    mutations = [mutation for mutation in MUTATIONS if args.match in mutation.name]

    root = Path(__file__).resolve().parent.parent
    deps = args.deps.resolve()
    problems = caught = 0

    with tempfile.TemporaryDirectory(prefix="stapes-mutants-") as temp:
        work = Path(temp) / "repo"
        prepare(root, deps, work)

        for mutation in mutations:
            target = work / mutation.path
            source = target.read_text(encoding="utf-8")
            if source.count(mutation.original) != 1:
                print(f"STALE     {mutation.name}: pattern no longer matches exactly once")
                problems += 1
                continue

            target.write_text(source.replace(mutation.original, mutation.replacement),
                              encoding="utf-8")
            if run(["cmake", "--build", "build"], work).returncode != 0:
                print(f"NO BUILD  {mutation.name}")
                problems += 1
            elif tests_catch_it(work):
                print(f"caught    {mutation.name}", flush=True)
                caught += 1
            else:
                print(f"SURVIVED  {mutation.name}", flush=True)
                problems += 1
            target.write_text(source, encoding="utf-8")

    print(f"{caught} of {len(mutations)} injected bugs caught")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
