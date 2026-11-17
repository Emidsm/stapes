"""A gamepad read straight from Linux, so the hands-on mode does not depend on the browser.

Browsers map a controller that is not a known Xbox or PlayStation model by guesswork. One
generic controller, under Firefox on Linux, came through with its right stick as the triggers
and a trigger fully pressed until first touched. The kernel reports the same controller exactly,
as evdev events with fixed codes. This module reads them with the standard library only, and
hands the page the state in the browser's standard layout, so the page's mapping is unchanged.

Linux only. Elsewhere, or with no controller, the page falls back to the browser's Gamepad API.
"""

import fcntl
import os
import re
import select
import struct
import threading
from dataclasses import dataclass, field
from pathlib import Path

EV_KEY = 0x01
EV_ABS = 0x03
EVENT_FORMAT = "llHHi"  # struct input_event on 64-bit Linux: time, type, code, value.
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

# Linux button codes, in the order of the browser's standard layout. X is the left face button
# and Y the top one, wherever a controller prints its letters.
BUTTONS = (
    0x130,  # 0  BTN_SOUTH: A
    0x131,  # 1  BTN_EAST: B
    0x134,  # 2  BTN_WEST: X
    0x133,  # 3  BTN_NORTH: Y
    0x136,  # 4  BTN_TL: LB
    0x137,  # 5  BTN_TR: RB
    0x138,  # 6  BTN_TL2: LT
    0x139,  # 7  BTN_TR2: RT
    0x13A,  # 8  BTN_SELECT: View
    0x13B,  # 9  BTN_START: Menu
    0x13D,  # 10 BTN_THUMBL
    0x13E,  # 11 BTN_THUMBR
)
BTN_MODE = 0x13C  # Home, standard index 16.

ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
ABS_GAS, ABS_BRAKE, ABS_HAT0X, ABS_HAT0Y = 0x09, 0x0A, 0x10, 0x11
ABS_CODES = (ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ, ABS_GAS, ABS_BRAKE, ABS_HAT0X,
             ABS_HAT0Y)


@dataclass(frozen=True)
class AxisRange:
    minimum: int
    maximum: int

    def centred(self, value: int) -> float:
        """A stick axis from -1 to 1."""
        half = (self.maximum - self.minimum) / 2
        return max(-1.0, min(1.0, (value - self.minimum - half) / half))

    def pressed(self, value: int) -> float:
        """A trigger from 0 to 1."""
        return max(0.0, min(1.0, (value - self.minimum) / (self.maximum - self.minimum)))


@dataclass
class GamepadState:
    """What the controller holds now, from its events. Independent of any device file."""

    name: str
    ranges: dict[int, AxisRange]
    keys: dict[int, int] = field(default_factory=dict)
    axes: dict[int, int] = field(default_factory=dict)
    # Buttons pressed since the last snapshot, so a press shorter than a frame still counts.
    pressed_since: set[int] = field(default_factory=set)

    def apply(self, event_type: int, code: int, value: int) -> None:
        if event_type == EV_KEY:
            self.keys[code] = value
            if value:
                self.pressed_since.add(code)
        elif event_type == EV_ABS:
            self.axes[code] = value

    def _stick(self, code: int) -> float:
        axis_range = self.ranges.get(code)
        if axis_range is None or code not in self.axes:
            return 0.0
        return axis_range.centred(self.axes[code])

    def _trigger(self, code: int) -> float | None:
        axis_range = self.ranges.get(code)
        if axis_range is None or code not in self.axes:
            return None
        return axis_range.pressed(self.axes[code])

    def snapshot(self) -> dict:
        """The state in the browser's standard layout, and the presses seen since last time."""
        # An Xbox-style controller (xpad) puts the right stick on RX and RY and the triggers on
        # Z and RZ. A generic HID controller puts the right stick on Z and RZ and the triggers on
        # GAS and BRAKE.
        xpad_layout = ABS_RX in self.ranges
        right = (ABS_RX, ABS_RY) if xpad_layout else (ABS_Z, ABS_RZ)
        triggers = (ABS_Z, ABS_RZ) if xpad_layout else (ABS_BRAKE, ABS_GAS)

        def button(code: int) -> dict:
            down = bool(self.keys.get(code, 0)) or code in self.pressed_since
            return {"pressed": down, "value": 1.0 if down else 0.0}

        buttons = [button(code) for code in BUTTONS]
        for index, code in ((6, triggers[0]), (7, triggers[1])):
            value = self._trigger(code)
            if value is not None:
                buttons[index] = {"pressed": value > 0.5, "value": value}
        hat_x, hat_y = self.axes.get(ABS_HAT0X, 0), self.axes.get(ABS_HAT0Y, 0)
        for down in (hat_y < 0, hat_y > 0, hat_x < 0, hat_x > 0):  # 12-15: up, down, left, right
            buttons.append({"pressed": down, "value": 1.0 if down else 0.0})
        buttons.append(button(BTN_MODE))
        self.pressed_since.clear()
        return {
            "id": f"{self.name} (read from Linux)",
            "mapping": "standard",
            "buttons": buttons,
            "axes": [self._stick(ABS_X), self._stick(ABS_Y), self._stick(right[0]),
                     self._stick(right[1])],
        }


def _eviocgabs(code: int) -> int:
    # _IOR('E', 0x40 + code, struct input_absinfo), and struct input_absinfo is six ints.
    return (2 << 30) | (24 << 16) | (ord("E") << 8) | (0x40 + code)


def read_axes(fd: int) -> tuple[dict[int, AxisRange], dict[int, int]]:
    """The axes the device has, with their ranges and current values. An axis it lacks reports
    an empty range."""
    ranges, values = {}, {}
    for code in ABS_CODES:
        try:
            value, minimum, maximum, *_ = struct.unpack(
                "6i", fcntl.ioctl(fd, _eviocgabs(code), bytes(24)))
        except OSError:
            continue
        if maximum > minimum:
            ranges[code] = AxisRange(minimum, maximum)
            values[code] = value
    return ranges, values


def find_gamepads(devices: Path = Path("/proc/bus/input/devices")) -> list[tuple[str, Path]]:
    """Every input device with a joystick handler: its name and its event file."""
    found = []
    try:
        text = devices.read_text(encoding="utf-8")
    except OSError:
        return found
    for block in text.split("\n\n"):
        name = re.search(r'^N: Name="(.*)"$', block, re.MULTILINE)
        handlers = re.search(r"^H: Handlers=(.*)$", block, re.MULTILINE)
        if not name or not handlers:
            continue
        words = handlers.group(1).split()
        event = next((word for word in words if word.startswith("event")), None)
        if event and any(word.startswith("js") for word in words):
            found.append((name.group(1), Path("/dev/input") / event))
    return found


class LinuxGamepad:
    """Reads the first gamepad Linux knows of, on a background thread."""

    def __init__(self, name: str, path: Path):
        self.path = path
        self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self._lock = threading.Lock()
        ranges, values = read_axes(self._fd)
        self._state = GamepadState(name, ranges, axes=values)
        self._closed = False
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    @classmethod
    def open_first(cls) -> "LinuxGamepad | None":
        for name, path in find_gamepads():
            try:
                return cls(name, path)
            except OSError:
                continue  # Not readable: the user is not in the `input` group, for one.
        return None

    def _read(self) -> None:
        while not self._closed:
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.5)
                if not ready:
                    continue
                data = os.read(self._fd, EVENT_SIZE * 64)
            except OSError:
                return  # Unplugged. The page falls back to the browser.
            with self._lock:
                for offset in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                    _, _, event_type, code, value = struct.unpack_from(EVENT_FORMAT, data, offset)
                    self._state.apply(event_type, code, value)

    @property
    def connected(self) -> bool:
        return self._thread.is_alive()

    def snapshot(self) -> dict | None:
        if not self.connected:
            return None
        with self._lock:
            return self._state.snapshot()

    def close(self) -> None:
        self._closed = True
        self._thread.join(timeout=1.0)
        os.close(self._fd)
