"""The gamepad read from Linux: events in, the browser's standard layout out."""

import pytest

from stapes import gamepad
from stapes.gamepad import EV_ABS, EV_KEY, AxisRange, GamepadState

FULL = AxisRange(0, 255)
# A generic HID controller, as one reported itself: right stick on Z and RZ, triggers on GAS and
# BRAKE, no RX or RY.
GENERIC = {code: FULL for code in (0x00, 0x01, 0x02, 0x05, 0x09, 0x0A)}
# An Xbox-style controller under xpad: right stick on RX and RY, triggers on Z and RZ.
XPAD = {0x00: AxisRange(-32768, 32767), 0x01: AxisRange(-32768, 32767),
        0x03: AxisRange(-32768, 32767), 0x04: AxisRange(-32768, 32767),
        0x02: AxisRange(0, 1023), 0x05: AxisRange(0, 1023)}


@pytest.mark.requirement("SRS-PY-007")
def test_a_generic_controller_maps_to_the_standard_layout():
    state = GamepadState("pad", GENERIC)
    for code in (0x00, 0x01, 0x02, 0x05):
        state.apply(EV_ABS, code, 128)  # Every stick centred.
    state.apply(EV_ABS, 0x05, 0)        # Right stick fully up.
    state.apply(EV_ABS, 0x09, 255)      # Right trigger fully pressed.
    state.apply(EV_KEY, 0x134, 1)       # The left face button: X.
    snapshot = state.snapshot()
    assert snapshot["mapping"] == "standard"
    assert snapshot["axes"][0] == pytest.approx(0.0, abs=0.01)
    assert snapshot["axes"][3] == pytest.approx(-1.0)
    assert snapshot["buttons"][7] == {"pressed": True, "value": 1.0}
    assert snapshot["buttons"][6]["value"] == 0.0
    assert snapshot["buttons"][2]["pressed"]
    assert len(snapshot["buttons"]) == 17


@pytest.mark.requirement("SRS-PY-007")
def test_an_xpad_controller_maps_its_right_stick_and_triggers():
    state = GamepadState("pad", XPAD)
    state.apply(EV_ABS, 0x03, 32767)  # Right stick fully right.
    state.apply(EV_ABS, 0x02, 1023)   # Left trigger fully pressed.
    snapshot = state.snapshot()
    assert snapshot["axes"][2] == pytest.approx(1.0)
    assert snapshot["buttons"][6]["value"] == 1.0


@pytest.mark.requirement("SRS-PY-007")
def test_a_press_shorter_than_a_frame_is_seen_once():
    state = GamepadState("pad", GENERIC)
    state.apply(EV_KEY, 0x130, 1)
    state.apply(EV_KEY, 0x130, 0)
    assert state.snapshot()["buttons"][0]["pressed"]
    assert not state.snapshot()["buttons"][0]["pressed"]


@pytest.mark.requirement("SRS-PY-007")
def test_gamepads_are_found_by_their_joystick_handler(tmp_path):
    devices = tmp_path / "devices"
    devices.write_text(
        'I: Bus=0003\nN: Name="Some Pad"\nH: Handlers=event14 js0 \n\n'
        'I: Bus=0003\nN: Name="Some Pad Consumer Control"\nH: Handlers=kbd event16 \n\n',
        encoding="utf-8")
    found = gamepad.find_gamepads(devices)
    assert [(name, str(path)) for name, path in found] == [("Some Pad", "/dev/input/event14")]
