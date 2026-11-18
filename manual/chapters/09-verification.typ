#import "/template.typ": *

#chapter("How the claims are checked", tag: "verification", summary: [
  A number in this manual is only as good as the test behind it, so the tests are tested too.
])

== Requirements and traceability

Every requirement has an identifier such as `SRS-SAF-004`, a statement a test can decide, and the
files that implement it. Every test declares the requirements it verifies. The traceability
matrix is then _generated_ from the test results.

The build fails if a requirement has no passing test, if a test names a requirement that does
not exist, or if a test declares none. At the time this manual was built there were
#data.requirements requirements and #data.test_cases test cases, #data.passed of them passing.

== Risk controls

The risk analysis lists #data.hazards hazards. The control for each is a list of requirement
identifiers, never a sentence, and the same tool fails if a hazard has no control or a control
is not verified by passing tests. The method is structured after ISO 14971; no compliance with
it is claimed.

== Measuring the truth

The simulator knows the true position of the tool. The controller and the monitor do not: they
see only sensors. Every accuracy figure in this manual is measured on the true state, by test
code that nothing in the running system can reach. A controller cannot look good by agreeing
with its own sensors.

== Testing the tests

Line coverage shows that a line ran. It does not show that a test would notice if the line were
wrong. `tools/mutation_check.py` holds a list of known bugs, each one a single change to a single
line: a reversed sign, a dropped term, a limit off by one. It applies them one at a time, and
fails if the tests still pass.

#why[
  When the safety monitor's bugs were added to the list, two survived the tests that had been
  written as its specification. A timestamp more than $2^63$ ticks in the future wrapped round
  into a small age and was accepted. And removing the check on the tip's distance from the line
  went unnoticed, because every test pose that failed it also failed the check on the shaft. Both
  functions had full line coverage.
]

The tool also fails if a pattern in its list no longer matches the code, so the list cannot
silently go stale.

== Determinism

The same seed gives the same run, bit for bit. A hash of the entire state is compared:

- between two runs in one process;
- between two separate processes, which catches reads of uninitialised memory and any dependence
  on addresses;
- between GCC and Clang, in CI;
- between C++ and Python: a run stepped through the bindings must give the hash a C++ program
  prints.

This is why a failure can always be reproduced from its seed. The claim is limited to what is
tested: a given compiler and maths library.

== No allocation in a tick

Allocating memory takes an unpredictable time, which a control loop cannot afford. A test
replaces the global `operator new` with a counter and runs complete ticks, with a fault active,
through a safety stop and with a trace being recorded. The count must stay at zero.

== Continuous integration

Every push builds and tests with GCC and Clang, in debug, release and sanitizer builds. Separate
jobs check formatting and static analysis at pinned tool versions, measure coverage, compare
the two compilers' hashes, run the injected bugs, and run the benchmarks. One job builds
everything including the Python layer, and produces the complete traceability matrix and the
test report.
