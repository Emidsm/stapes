"""The plots, the viewer and the manual use one palette, defined in three file formats."""

import json
import re
from pathlib import Path

import pytest

from stapes import palette

ROOT = Path(__file__).resolve().parents[2]


def _viewer_colours() -> dict[str, str]:
    css = (ROOT / "viewer" / "style.css").read_text(encoding="utf-8")
    return {name: value.upper() for name, value in re.findall(r"--([a-z-]+):\s*(#[0-9a-fA-F]{6});", css)}


@pytest.mark.requirement("SRS-PY-005")
def test_the_three_palettes_hold_the_same_values():
    manual = json.loads((ROOT / "manual" / "tokens.json").read_text(encoding="utf-8"))
    viewer = _viewer_colours()
    plots = {
        "ink": palette.INK,
        "teal": palette.TEAL,
        "pale-teal": palette.PALE_TEAL,
        "signal-red": palette.SIGNAL_RED,
        "muted": palette.MUTED,
        "rule": palette.RULE,
    }
    for name, value in plots.items():
        assert manual[name].upper() == value.upper(), name
        assert viewer[name] == value.upper(), name


@pytest.mark.requirement("SRS-PY-005")
def test_the_palette_is_the_documented_one():
    assert palette.INK == "#0B1F2A"
    assert palette.TEAL == "#0F766E"
    assert palette.PALE_TEAL == "#CCFBF1"
    assert palette.SIGNAL_RED == "#D92D20"
