"""Command line for the scenario layer: `python -m stapes --help`."""

import argparse
import sys
from pathlib import Path

import stapes
from stapes import viewer_export
from stapes.scenarios import SCENARIOS


def _list(_args) -> int:
    width = max(len(name) for name in SCENARIOS)
    for name, spec in SCENARIOS.items():
        print(f"{name:<{width}}  {spec.title}")
    return 0


def _run(args) -> int:
    trace = stapes.run(SCENARIOS[args.scenario], args.seed)
    print(f"scenario        {trace.spec.name} (seed {trace.seed})")
    print(f"core            {stapes.__version__}, {stapes.compiler}")
    print(f"max tip error   {trace.max_radial_m * 1e6:.1f} um from the planned line")
    print(f"max axis offset {trace.max_axis_offset_m * 1e6:.1f} um at the entry")
    print(f"final depth     {trace.depth_m[-1] * 1e3:.3f} mm")
    if trace.stop_tick is None:
        print("monitor         no stop")
    else:
        print(f"monitor         stopped at tick {trace.stop_tick}: {trace.cause_name}")
        if trace.detection_ticks is not None:
            print(f"detection       {trace.detection_ticks} ms after the fault began")
    return 0


def _export(args) -> int:
    for path in viewer_export.export_all(args.directory, args.seed):
        print(path)
    return 0


def _plots(args) -> int:
    from stapes import plots  # Imported here so the other commands do not need matplotlib.

    for path in plots.write_all(args.directory, args.seed, args.format):
        print(path)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m stapes", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    commands.add_parser("list", help="list the named scenarios").set_defaults(run=_list)

    run_parser = commands.add_parser("run", help="run one scenario and print what happened")
    run_parser.add_argument("scenario", choices=list(SCENARIOS))
    run_parser.add_argument("--seed", type=int, default=1)
    run_parser.set_defaults(run=_run)

    export_parser = commands.add_parser("export", help="write every scenario for the viewer")
    export_parser.add_argument("directory", type=Path)
    export_parser.add_argument("--seed", type=int, default=1)
    export_parser.set_defaults(run=_export)

    plots_parser = commands.add_parser("plots", help="write the figures for every scenario")
    plots_parser.add_argument("directory", type=Path)
    plots_parser.add_argument("--seed", type=int, default=1)
    plots_parser.add_argument("--format", choices=["svg", "png", "pdf"], default="svg")
    plots_parser.set_defaults(run=_plots)

    args = parser.parse_args(argv)
    return args.run(args)


if __name__ == "__main__":
    sys.exit(main())
