#!/usr/bin/env python3
"""Build the operation manual: figures and measured values first, then the PDF.

The manual quotes no number by hand. This script writes `manual/build/data.json` from the test
results and the figures from recorded runs, and Typst reads both. Fonts are the ones in
`manual/fonts`, and system fonts are ignored, so the PDF is the same on every machine.

Needs Typst, and the Python layer: run it with the built module on PYTHONPATH.
"""

import argparse
import datetime
import json
import shutil
import subprocess
import sys
import tomllib
from pathlib import Path

from test_report import build_environment, recorded_values
from traceability import load_requirements, load_test_cases

ROOT = Path(__file__).resolve().parent.parent
MANUAL = ROOT / "manual"


def project_version() -> str:
    for line in (ROOT / "CMakeLists.txt").read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("VERSION "):
            return line.split()[1]
    raise ValueError("no project version in CMakeLists.txt")


def commit() -> str:
    try:
        short = subprocess.run(["git", "rev-parse", "--short=10", "HEAD"], cwd=ROOT,
                               capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True,
                               text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    # Honest about what is not in the commit.
    return f"{short} with uncommitted changes" if dirty else short


def commit_date() -> str:
    """The date of the commit the manual is built from, so one commit always gives one PDF."""
    try:
        return subprocess.run(["git", "log", "-1", "--format=%cs"], cwd=ROOT,
                              capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return datetime.date.today().isoformat()


def describe(environment: list[str]) -> str:
    """The build as a phrase: 'GNU 16.2.1 in a Release build'."""
    values = dict(line.removeprefix("- ").split(": ", 1) for line in environment)
    if "Compiler" not in values:
        return "an unrecorded compiler"
    build = f" in a {values['Build type']} build" if "Build type" in values else ""
    return values["Compiler"] + build


def write_data(results: Path, build_dir: Path, target: Path) -> None:
    cases = load_test_cases(results)
    if not cases:
        raise ValueError(f"no test results in {results}")
    environment = build_environment(build_dir)
    with (ROOT / "docs/risk/hazards.toml").open("rb") as file:
        hazards = tomllib.load(file)["hazard"]
    data = {
        "version": project_version(),
        "compiler": describe(environment),
        "requirements": len(load_requirements(ROOT / "docs/requirements/requirements.toml")),
        "hazards": len(hazards),
        "test_cases": len(cases),
        "passed": sum(case.passed for case in cases),
        "measurements": recorded_values(results),
    }
    target.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=ROOT / "build/python/test-results")
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build/python")
    parser.add_argument("--viewer-image", type=Path, default=ROOT / "docs/media/viewer.png")
    parser.add_argument("--output", type=Path, default=MANUAL / "build/stapes-manual.pdf")
    args = parser.parse_args()

    build = MANUAL / "build"
    figures = build / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    write_data(args.results, args.build_dir, build / "data.json")

    from stapes import plots  # Needs the built module, so it is imported only here.

    plots.write_all(figures)
    shutil.copyfile(args.viewer_image, figures / "viewer.png")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["typst", "compile", "--root", str(MANUAL), "--font-path", str(MANUAL / "fonts"),
         "--ignore-system-fonts", "--input", f"commit={commit()}",
         "--input", f"date={commit_date()}",
         str(MANUAL / "manual.typ"), str(args.output)],
        check=True)
    print(args.output)
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(ROOT / "python"))
    sys.exit(main())
