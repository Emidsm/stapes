"""Figures for the report and the manual, drawn from recorded runs."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # Draws to files. No display is needed, so this runs in CI.
import matplotlib.pyplot as plt
import numpy as np

from . import palette
from .run import Trace, run
from .scenarios import SCENARIOS

DESIGN_TARGET_UM = 200.0


def _style(axes) -> None:
    for side in ("top", "right"):
        axes.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        axes.spines[side].set_color(palette.MUTED)
    axes.tick_params(colors=palette.MUTED, labelsize=8)
    axes.grid(True, color=palette.RULE, linewidth=0.6)
    axes.set_axisbelow(True)


def _mark_events(axes, trace: Trace) -> None:
    """A fault and a stop are the only things drawn in red."""
    if trace.fault_start_tick is not None:
        axes.axvline(trace.fault_start_tick * trace.period_s, color=palette.SIGNAL_RED,
                     linewidth=1.0, linestyle=(0, (4, 3)))
    if trace.stop_tick is not None:
        axes.axvline(trace.stop_tick * trace.period_s, color=palette.SIGNAL_RED, linewidth=1.4)


def radial_error_figure(trace: Trace):
    """Distance of the true tip from the planned line, over the run."""
    figure, axes = plt.subplots(figsize=(6.4, 2.9), constrained_layout=True)
    # Only where the monitor holds the tip to the line. During an approach the tip is
    # centimetres away by design, and the gap in the line shows where the corridor starts.
    radial_um = np.where(trace.inside_guard, trace.radial_m * 1e6, np.nan)
    axes.plot(trace.time_s, radial_um, color=palette.TEAL, linewidth=1.3)
    axes.axhline(DESIGN_TARGET_UM, color=palette.INK, linewidth=0.8, linestyle=(0, (1, 2)))
    axes.annotate("0.2 mm design target", (trace.time_s[-1], DESIGN_TARGET_UM),
                  xytext=(0, 3), textcoords="offset points", ha="right", fontsize=7.5,
                  color=palette.INK)
    if trace.spec.monitored:
        corridor_um = trace.scenario.safety.corridor_radius_m * 1e6
        axes.axhline(corridor_um, color=palette.MUTED, linewidth=0.8)
        axes.annotate("monitor corridor", (trace.time_s[-1], corridor_um), xytext=(0, 3),
                      textcoords="offset points", ha="right", fontsize=7.5, color=palette.MUTED)
        axes.set_ylim(0.0, max(1.25 * corridor_um, 1.1 * float(np.nanmax(radial_um))))
    else:
        axes.set_yscale("log")
    _mark_events(axes, trace)
    axes.set_xlim(0.0, float(trace.time_s[-1]))
    axes.set_xlabel("time (s)", fontsize=8.5, color=palette.INK)
    axes.set_ylabel("tip to planned line (µm)", fontsize=8.5, color=palette.INK)
    axes.set_title(trace.spec.title, loc="left", fontsize=10, color=palette.INK,
                   fontweight="bold")
    _style(axes)
    return figure


def depth_figure(trace: Trace):
    """Planned and true depth along the tunnel."""
    figure, axes = plt.subplots(figsize=(6.4, 2.6), constrained_layout=True)
    axes.plot(trace.time_s, trace.planned_depth_m * 1e3, color=palette.MUTED, linewidth=1.0,
              linestyle=(0, (4, 2)), label="planned")
    axes.plot(trace.time_s, trace.depth_m * 1e3, color=palette.TEAL, linewidth=1.3, label="true")
    _mark_events(axes, trace)
    axes.set_xlim(0.0, float(trace.time_s[-1]))
    axes.set_xlabel("time (s)", fontsize=8.5, color=palette.INK)
    axes.set_ylabel("depth along the tunnel (mm)", fontsize=8.5, color=palette.INK)
    axes.legend(frameon=False, fontsize=8, labelcolor=palette.INK, loc="upper left")
    _style(axes)
    return figure


def containment_figure(contained: Trace, unmonitored: Trace):
    """The same fault with and without the monitor, on one logarithmic axis."""
    figure, axes = plt.subplots(figsize=(6.4, 3.1), constrained_layout=True)
    floor_um = 1.0
    axes.plot(unmonitored.time_s, np.maximum(unmonitored.radial_m * 1e6, floor_um),
              color=palette.MUTED, linewidth=1.2, label="monitor disabled")
    axes.plot(contained.time_s, np.maximum(contained.radial_m * 1e6, floor_um),
              color=palette.TEAL, linewidth=1.5, label="monitor active")
    axes.axhline(500.0, color=palette.INK, linewidth=0.8, linestyle=(0, (1, 2)))
    axes.annotate("0.5 mm harm limit", (contained.time_s[-1], 500.0), xytext=(0, 3),
                  textcoords="offset points", ha="right", fontsize=7.5, color=palette.INK)
    _mark_events(axes, contained)
    axes.set_yscale("log")
    axes.set_xlim(0.0, float(contained.time_s[-1]))
    axes.set_xlabel("time (s)", fontsize=8.5, color=palette.INK)
    axes.set_ylabel("tip to planned line (µm)", fontsize=8.5, color=palette.INK)
    axes.set_title("A frozen motor encoder, with and without the monitor", loc="left",
                   fontsize=10, color=palette.INK, fontweight="bold")
    axes.legend(frameon=False, fontsize=8, labelcolor=palette.INK, loc="upper left")
    _style(axes)
    return figure


def write_all(directory: Path, seed: int = 1, extension: str = "svg") -> list[Path]:
    """Runs every scenario and writes its figures."""
    directory.mkdir(parents=True, exist_ok=True)
    traces = {name: run(spec, seed) for name, spec in SCENARIOS.items()}
    # A run stopped on its way in never reaches the corridor, so it has no error to plot.
    figures = {f"radial-{name}": radial_error_figure(trace) for name, trace in traces.items()
               if trace.inside_guard.any()}
    figures["depth-nominal"] = depth_figure(traces["nominal"])
    figures["depth-frozen-motor-encoder"] = depth_figure(traces["frozen-motor-encoder"])
    figures["containment"] = containment_figure(traces["frozen-motor-encoder"],
                                                traces["frozen-motor-encoder-unmonitored"])
    written = []
    for name, figure in figures.items():
        path = directory / f"{name}.{extension}"
        # A fixed hash salt and no date keep the SVG identical between runs.
        figure.savefig(path, metadata={"Date": None} if extension == "svg" else None)
        plt.close(figure)
        written.append(path)
    return written
