#import "/template.typ": *

#chapter("Building and testing", tag: "build", summary: [
  One command configures, builds and runs every test. Nothing has to be installed beyond a
  compiler, CMake and Ninja.
])

== What you need

- CMake 3.28 or newer, and Ninja.
- A C++20 compiler. GCC and Clang are both tested.
- Python 3.10 or newer, only for the tools and for the optional Python layer.

Eigen, GoogleTest, Google Benchmark and pybind11 are downloaded by CMake at configure time. Each
is pinned to a version and checked against a hash, so every build uses the same bytes.

== Build presets

```sh
cmake --workflow --preset debug
```

A workflow preset configures, builds and tests in one step. There is one for each purpose:

#data-table(
  (auto, 1fr),
  [Preset], [What it is for],
  [`debug`], [Day-to-day work. Compiled at `-O1`, because unoptimised Eigen makes the tests take minutes.],
  [`release`], [Full optimisation. The benchmarks are meaningful only here.],
  [`sanitize`], [AddressSanitizer and UndefinedBehaviorSanitizer on everything, dependencies included.],
  [`coverage`], [GCC coverage instrumentation.],
  [`python`], [Release, plus the Python bindings and their tests. The only preset that needs Python packages.],
)

Every preset treats warnings as errors. To build with a second compiler, give it its own
directory:

```sh
CXX=clang++ cmake --preset debug -B build/clang-debug
cmake --build build/clang-debug
ctest --test-dir build/clang-debug
```

== The Python layer

The Python layer runs named scenarios and produces the files for the viewer and the figures in
this manual. It needs a virtual environment with three pinned packages:

#procedure[Set up and test the Python layer][
  + Create the environment and install the pinned packages:
    ```sh
    python3 -m venv .venv
    .venv/bin/pip install -r python/requirements.txt
    ```
  + Activate it, so that CMake finds its interpreter:
    ```sh
    source .venv/bin/activate
    ```
  + Build and test everything, C++ and Python:
    ```sh
    cmake --workflow --preset python
    ```
  + To use the package from a shell, point Python at the built module and at the package:
    ```sh
    export PYTHONPATH=build/python/python:python
    python -m stapes list
    ```
]

== The verification tools

Three tools turn test results into evidence. Each reads the JSON files the tests write to
`build/<preset>/test-results`.

```sh
python3 tools/traceability.py
  --requirements docs/requirements/requirements.toml
  --hazards docs/risk/hazards.toml
  --results build/python/test-results
  --output build/python/traceability.md
```

This builds the traceability matrix, and fails if a requirement has no passing test or a hazard
has no verified control. With a preset other than `python`, add `--skip-layer python`: those
builds do not run the Python tests, and the tool will not pretend they did.

```sh
python3 tools/test_report.py
  --requirements docs/requirements/requirements.toml
  --results build/python/test-results
  --build-dir build/python
  --output build/python/test-report.md
```

This builds the test report, with every value the tests measured.

```sh
python3 tools/mutation_check.py
```

This injects known bugs one at a time into a temporary copy of the code and fails if the tests
miss one. It takes a long time, since it rebuilds and tests once per bug. `--file safety` or
`--match "debounce"` narrows it to part of the list.

== If something goes wrong

#data-table(
  (38%, 1fr),
  [Symptom], [What to do],
  [CMake cannot download a dependency], [Check network access to GitHub and GitLab. A hash mismatch means the download was altered or truncated: delete `build/<preset>/_deps` and configure again.],
  [`import stapes_core` fails], [`PYTHONPATH` must contain both `build/python/python` and `python`. The module is built for one Python version: use the interpreter of the environment that was active at configure time.],
  [`test_realtime` is missing in a sanitizer build], [Expected. That test replaces the global `operator new`, and so does AddressSanitizer. Every other build runs it.],
  [The traceability tool reports `SRS-PY` requirements with no test], [The results come from a build without the Python tests. Use the `python` preset, or pass `--skip-layer python`.],
)
