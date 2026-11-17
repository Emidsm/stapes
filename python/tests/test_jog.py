"""The hands-on mode's Python side: recordings that replay, and the local server."""

import json
import threading
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from stapes import jog

ROOT = Path(__file__).resolve().parents[2]


def scripted_session(fault: str = "stuck_actuator") -> jog.JogSession:
    session = jog.JogSession(seed=3, fault=fault)
    across = np.cross(session.scenario.trajectory.direction, [0.0, 0.0, 1.0])
    across /= np.linalg.norm(across)
    for _ in range(40):
        session.advance(16, [*(0.03 * across), 0.0, False, False, False, 0, 0.0])
    for _ in range(150):
        session.advance(16, [0.0, 0.0, 0.0, 0.0, False, False, False, 0, 0.0])
    session.advance(16, [0.0, 0.0, 0.0, 0.0, True, False, False, 0, 0.0])
    for frame in range(300):
        session.advance(16, [0.0, 0.0, 0.0, 0.8, False, frame > 20, False, 0, 0.0])
    return session


@pytest.mark.requirement("SRS-PY-006")
def test_a_recorded_session_replays_to_the_same_hash():
    session = scripted_session()
    recording = json.loads(json.dumps(session.recording()))
    replayed = jog.replay(recording).recording()
    assert replayed["state_hash"] == recording["state_hash"]
    assert replayed["ticks"] == recording["ticks"]
    # Runs of the same input are stored once.
    assert len(recording["inputs"]) < 10


@pytest.mark.requirement("SRS-PY-006")
def test_a_recording_with_a_gap_is_refused():
    recording = scripted_session().recording()
    recording["inputs"][1][0] += 1
    with pytest.raises(ValueError, match="skip"):
        jog.replay(recording)


@pytest.mark.requirement("SRS-PY-006")
def test_a_session_in_joint_mode_replays_to_the_same_hash():
    session = jog.JogSession()
    session.advance(1, [0, 0, 0, 0, False, False, True, 0, 0])
    for _ in range(100):
        session.advance(16, [0, 0, 0, 0, False, False, False, 3, 1.0])
    assert jog.jog_mode_name(session.sim.mode) == "joint"
    recording = session.recording()
    assert jog.replay(recording).recording()["state_hash"] == recording["state_hash"]


@pytest.mark.requirement("SRS-PY-006")
def test_an_input_of_the_wrong_length_is_refused():
    with pytest.raises(ValueError, match="values"):
        jog.JogSession().advance(1, [0, 0, 0, 0, False, False])


@pytest.mark.requirement("SRS-PY-006")
def test_one_request_advances_at_most_the_cap():
    session = jog.JogSession()
    session.advance(10_000, [0.0, 0.0, 0.0, 0.0, False, False, False, 0, 0.0])
    assert session.sim.ticks == jog.MAX_TICKS_PER_REQUEST
    session.advance(-5, [0.0, 0.0, 0.0, 0.0, False, False, False, 0, 0.0])
    assert session.sim.ticks == jog.MAX_TICKS_PER_REQUEST


@pytest.mark.requirement("SRS-PY-006")
def test_the_server_serves_the_page_and_steps_the_simulation(tmp_path):
    server = jog._Server(("127.0.0.1", 0), ROOT / "viewer", tmp_path, seed=1)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"

    def post(path, body):
        request = urllib.request.Request(base + path, data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request) as response:
            return json.loads(response.read())

    try:
        with urllib.request.urlopen(base + "/jog.html") as response:
            assert b"<canvas" in response.read()
        with urllib.request.urlopen(base + "/api/jog/scene") as response:
            opened = json.loads(response.read())
        assert opened["state"]["tick"] == 0
        assert len(opened["state"]["chain_m"]) == 7
        state = post("/api/jog/step", {"ticks": 20, "input": [0, 0, 0.01, 0, False, False, False, 0, 0]})
        assert state["tick"] == 20
        assert state["mode"] == "free"
        reset = post("/api/jog/reset", {"fault": "frozen_tracker"})
        assert reset["scene"]["fault"] == "frozen_tracker"
        assert reset["state"]["tick"] == 0
        # The session that was reset is on disk, and replays.
        (saved,) = tmp_path.glob("jog-*.json")
        recording = json.loads(saved.read_text(encoding="utf-8"))
        assert jog.replay(recording).recording()["state_hash"] == recording["state_hash"]
    finally:
        server.shutdown()
        server.server_close()
