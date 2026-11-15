"""Writes the test results in the JSON layout GoogleTest uses.

tools/traceability.py reads that layout, so the Python tests appear in the same matrix as the
C++ ones. A test names the requirements it verifies with `@pytest.mark.requirement("SRS-...")`.
"""

import json
import os
from pathlib import Path

import pytest

_results: list[dict] = []


def pytest_configure(config):
    config.addinivalue_line("markers", "requirement(ids): the requirement IDs this test verifies")


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    # A test has three phases. It is recorded once, and a failure in any phase fails it.
    if report.when == "setup":
        marker = item.get_closest_marker("requirement")
        _results.append({
            "name": item.name,
            "suite": item.module.__name__,
            "status": "RUN",
            "result": "COMPLETED",
            "requirement": ",".join(marker.args) if marker else "",
        })
    if report.failed:
        _results[-1].setdefault("failures", []).append({"failure": str(report.longrepr)})
    if report.skipped:
        _results[-1]["result"] = "SKIPPED"


def pytest_sessionfinish(session, exitstatus):
    target = os.environ.get("STAPES_TEST_RESULTS")
    if not target:
        return
    suites: dict[str, list[dict]] = {}
    for result in _results:
        suites.setdefault(result.pop("suite"), []).append(result)
    document = {"testsuites": [{"name": name, "testsuite": tests}
                               for name, tests in suites.items()]}
    path = Path(target)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
