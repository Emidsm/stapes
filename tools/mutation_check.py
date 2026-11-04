#!/usr/bin/env python3
"""Inject known bugs, one at a time, and check that the test suite catches each one.

Line coverage only shows that a line ran. This shows that a test would notice if the
line were wrong. It works on a temporary copy and never touches the working tree.

Exits non-zero if a bug survives or a pattern no longer matches the code, so the list
cannot silently go stale.
"""

import argparse
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
    Mutation("all encoders share one noise stream", ARM, "kEncoderStreamBase + Index",
             "kEncoderStreamBase + (Index * 0)"),
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
]

TEST_TIMEOUT_S = 300


def run(command: list[str], cwd: Path) -> subprocess.CompletedProcess:
    return subprocess.run(command, cwd=cwd, capture_output=True, text=True, check=False)


def tests_catch_it(work: Path) -> bool:
    try:
        result = subprocess.run(["ctest", "--test-dir", "build", "--timeout", "120"], cwd=work,
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
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    deps = args.deps.resolve()
    problems = caught = 0

    with tempfile.TemporaryDirectory(prefix="stapes-mutants-") as temp:
        work = Path(temp) / "repo"
        prepare(root, deps, work)

        for mutation in MUTATIONS:
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

    print(f"{caught} of {len(MUTATIONS)} injected bugs caught")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
