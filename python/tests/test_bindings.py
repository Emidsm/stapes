"""The bindings expose what the core holds, and add no behaviour of their own."""

import os
import subprocess

import numpy as np
import pytest

import stapes


@pytest.mark.requirement("SRS-PY-001")
def test_a_run_stepped_from_python_matches_the_same_run_in_cpp():
    # The probe is a C++ program that prints three hashes. The third is a run with a frozen
    # encoder and a safety stop: tests/support/fault_scenarios.hpp, fault_workload_hash.
    probe = os.environ.get("STAPES_DETERMINISM_PROBE")
    if not probe:
        pytest.skip("STAPES_DETERMINISM_PROBE is not set; run through ctest")
    expected = subprocess.run([probe], capture_output=True, text=True, check=True).stdout
    expected_hash = int(expected.strip().split("-")[2], 16)

    scenario = stapes.KeyholeScenario.reference(42)
    scenario.fault = stapes.Fault(stapes.FaultKind.frozen_motor_encoder, 500, 1, 0)
    sim = stapes.KeyholeSimulation(scenario)
    state_hash = stapes.StateHash()
    for _ in range(2000):
        sim.step()
        state_hash.add(sim)

    assert sim.stop_tick is not None  # The run really included a stop.
    assert state_hash.value == expected_hash


@pytest.mark.requirement("SRS-PY-001")
def test_a_different_seed_gives_a_different_hash():
    def hash_of(seed):
        sim = stapes.KeyholeSimulation(stapes.KeyholeScenario.reference(seed))
        state_hash = stapes.StateHash()
        for _ in range(200):
            sim.step()
            state_hash.add(sim)
        return state_hash.value

    assert hash_of(1) == hash_of(1)
    assert hash_of(1) != hash_of(2)


@pytest.mark.requirement("SRS-PY-002")
def test_the_reference_scenario_is_the_one_the_core_defines():
    scenario = stapes.KeyholeScenario.reference(7)
    assert scenario.seed == 7
    assert scenario.fault.kind == stapes.FaultKind.none
    assert scenario.trajectory.length_m == pytest.approx(10e-3, abs=1e-12)
    assert scenario.feed_speed_m_s == 5e-3
    assert scenario.safety.corridor_radius_m == 0.3e-3
    assert scenario.safety.period_s == 1e-3
    assert np.linalg.norm(scenario.trajectory.direction) == pytest.approx(1.0, abs=1e-12)


@pytest.mark.requirement("SRS-PY-002")
def test_a_fault_set_from_python_reaches_the_simulation():
    scenario = stapes.KeyholeScenario.reference(1)
    scenario.fault = stapes.Fault(kind=stapes.FaultKind.comms_delay, start_tick=5, delay_ticks=50)
    sim = stapes.KeyholeSimulation(scenario)
    assert not sim.fault_active
    for _ in range(10):
        sim.step()
    assert sim.fault_active
    assert sim.cause == stapes.StopCause.stale_data
    assert sim.stop_tick == 5
    assert sim.brakes_engaged
    assert stapes.stop_cause_name(sim.cause) == "stale_data"


@pytest.mark.requirement("SRS-PY-002")
def test_invalid_arguments_arrive_as_python_exceptions():
    scenario = stapes.KeyholeScenario.reference(1)
    scenario.fault = stapes.Fault(kind=stapes.FaultKind.stuck_actuator, joint=6)
    with pytest.raises(ValueError):
        stapes.KeyholeSimulation(scenario)
    with pytest.raises(ValueError):
        stapes.TraceRecorder(0)


@pytest.mark.requirement("SRS-PY-002")
def test_the_simulation_starts_on_the_plan_and_reports_the_true_pose():
    scenario = stapes.KeyholeScenario.reference(1)
    sim = stapes.KeyholeSimulation(scenario)
    pose = sim.true_tool_pose()
    assert pose.shape == (4, 4)
    assert scenario.trajectory.radial_distance(pose[:3, 3]) < 2e-6
    # The pose is forward kinematics of the true joint angles, and the chain ends at the tip.
    again = stapes.forward_kinematics(scenario.arm, sim.joint_positions)
    assert np.array_equal(pose, again)
    chain = stapes.chain_positions(scenario.arm, sim.joint_positions)
    assert chain.shape == (stapes.JOINT_COUNT + 1, 3)
    assert np.array_equal(chain[-1], pose[:3, 3])


@pytest.mark.requirement("SRS-PY-002")
def test_trace_columns_have_one_row_per_recorded_tick():
    sim = stapes.KeyholeSimulation(stapes.KeyholeScenario.reference(1))
    recorder = stapes.TraceRecorder(20)
    for _ in range(25):
        sim.step()
        recorder.record(sim)
    assert len(recorder) == 20
    assert recorder.dropped == 5
    assert recorder.tick().tolist() == list(range(1, 21))
    assert recorder.q().shape == (20, stapes.JOINT_COUNT)
    assert recorder.target().shape == (20, stapes.JOINT_COUNT)
    assert recorder.tip_m().shape == (20, 3)
    assert np.allclose(np.linalg.norm(recorder.tool_axis(), axis=1), 1.0, atol=1e-12)
    assert recorder.radial_m().shape == (20,)
