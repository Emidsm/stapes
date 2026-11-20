"""Each named scenario shows what its description says it shows."""

import json
import math

import numpy as np
import pytest

import stapes
from stapes import viewer_export
from stapes.scenarios import SCENARIOS

HARM_LIMIT_M = 0.5e-3
DESIGN_TARGET_M = 0.2e-3


@pytest.fixture(scope="module")
def traces():
    return {name: stapes.run(spec, seed=1) for name, spec in SCENARIOS.items()}


@pytest.mark.requirement("SRS-PY-003")
@pytest.mark.parametrize("name", list(SCENARIOS))
def test_scenario_has_its_expected_outcome(traces, name):
    trace = traces[name]
    spec = trace.spec
    if spec.approach:
        # Starts from rest, and ends once the feed has finished and settled.
        assert trace.approaching[0]
        assert spec.expect_stop or not trace.approaching[-1]
    else:
        assert len(trace.tick) == spec.ticks
        assert not trace.approaching.any()
    if spec.expect_stop:
        assert trace.stop_tick is not None
        assert trace.cause != stapes.StopCause.none
        assert trace.brakes_engaged[-1]
        assert trace.max_radial_m < HARM_LIMIT_M
    else:
        assert trace.stop_tick is None
        assert trace.cause == stapes.StopCause.none
        assert not trace.brakes_engaged.any()
    if spec.has_fault:
        start = spec.fault_start_tick
        assert trace.fault_start_tick == start
        assert not trace.fault_active[: start - 1].any()
        assert trace.fault_active[start:].all()
        if spec.expect_stop:
            assert trace.detection_ticks >= 0
    else:
        assert not trace.fault_active.any()
    if spec.monitored and not spec.expect_stop:
        assert trace.max_radial_m < DESIGN_TARGET_M


@pytest.mark.requirement("SRS-PY-003")
def test_the_monitor_is_what_contains_the_fault(traces):
    contained = traces["frozen-motor-encoder"]
    unmonitored = traces["frozen-motor-encoder-unmonitored"]
    assert contained.max_radial_m < HARM_LIMIT_M
    assert unmonitored.max_radial_m > 100 * HARM_LIMIT_M


@pytest.mark.requirement("SRS-PY-003")
def test_a_fault_on_the_way_in_is_stopped_before_the_corridor(traces):
    trace = traces["approach-stuck-actuator"]
    assert trace.approaching[trace.stop_tick]
    assert not trace.inside_guard.any()
    # Centimetres from the entry, as the description says.
    assert trace.depth_m[trace.stop_tick] < -10e-3


@pytest.mark.requirement("SRS-PY-003")
def test_stale_data_is_refused_in_the_tick_it_appears(traces):
    trace = traces["comms-delay"]
    assert trace.cause_name == "stale_data"
    assert trace.detection_ticks == 0


@pytest.mark.requirement("SRS-PY-004")
@pytest.mark.parametrize("name", list(SCENARIOS))
def test_viewer_document_is_complete_and_finite(traces, name):
    trace = traces[name]
    document = viewer_export.viewer_document(trace)
    # Refuses NaN and infinity, which are not JSON.
    text = json.dumps(document, allow_nan=False)
    loaded = json.loads(text)

    assert loaded["format"] == viewer_export.FORMAT_VERSION
    assert loaded["name"] == name
    ticks = len(trace.tick)
    for series in loaded["series"].values():
        assert len(series) == ticks
    frames = loaded["frames"]["chain_m"]
    assert len(frames) == math.ceil(ticks / viewer_export.FRAME_EVERY_TICKS)
    assert all(len(chain) == stapes.JOINT_COUNT + 1 for chain in frames)
    assert loaded["events"]["stop_tick"] == trace.stop_tick
    assert loaded["events"]["fault_start_tick"] == trace.fault_start_tick
    assert loaded["events"]["approach_end_tick"] == trace.approach_end_tick
    assert loaded["tunnel"]["length_mm"] == pytest.approx(10.0)

    # The view across the tunnel is the same distance the core measured.
    across = np.hypot(loaded["series"]["across_u_um"], loaded["series"]["across_v_um"])
    assert np.allclose(across, np.asarray(loaded["series"]["radial_um"]), atol=0.2)


@pytest.mark.requirement("SRS-PY-004")
def test_export_is_reproducible_and_indexed(tmp_path):
    first = viewer_export.export_all(tmp_path / "a", seed=3)
    second = viewer_export.export_all(tmp_path / "b", seed=3)
    assert [path.name for path in first] == [path.name for path in second]
    for a, b in zip(first, second):
        assert a.read_bytes() == b.read_bytes()

    index = json.loads((tmp_path / "a" / "index.json").read_text())
    assert [entry["name"] for entry in index["scenarios"]] == list(SCENARIOS)
    for entry in index["scenarios"]:
        assert (tmp_path / "a" / entry["file"]).is_file()
