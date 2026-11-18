#import "/template.typ": *

#chapter("Extending Stapes", tag: "extending", summary: [
  How to add to the project without weakening what it proves.
])

== The rule

A new piece of logic arrives with three things: a requirement, a test that verifies it, and an
injected bug that the test must catch. The tools enforce the first two. The third is a habit,
and it is the one that finds the gaps.

#procedure[Add a behaviour][
  + Write the requirement in `docs/requirements/requirements.toml`: an identifier in the right
    area, and a statement a test can decide.
  + Write the test first. Declare the requirement with `stapes::test::verifies("SRS-...")`. It
    should fail.
  + Write the code until the test passes.
  + Add one or more entries to `MUTATIONS` in `tools/mutation_check.py`: the ways this code could
    plausibly be wrong. Run the tool on that file. If a bug survives, the test is missing a case.
  + If the behaviour controls a hazard, add its identifier to that hazard in
    `docs/risk/hazards.toml`.
]

== Add a check to the monitor

+ Add a value to `StopCause`, in the position its priority deserves, and a name for it in
  `to_string`.
+ Add its limit to `SafetyConfig`, validate it in the constructor, and set a reference value.
+ Decide whether it can be noise. If not, it belongs with the immediate checks. If so, it is a
  debounced check and gets its own counter.
+ Measure its margin on a healthy run before choosing the limit, and add it to the test that
  measures the margins.
+ Expose the new cause in `bindings/src/stapes_core.cpp`.

#note[
  Never weaken a requirement to make a failing test pass before the cause of the failure is
  understood. A failing delay test once exposed fragile controller gains, and the gains were
  changed, not the requirement.
]

== Add a fault

+ Add a value to `FaultKind`.
+ Implement it in `FaultInjector::update` for a fault on readings, or in `command_torques` for a
  fault on commands. Use only fixed-size storage.
+ Test it in isolation, then add it to the campaign with the outcome it must have.
+ Add a `ScenarioSpec` in `python/stapes/scenarios.py` so that it can be run by name and appears
  in the viewer.

== Add a scenario

A scenario is a few lines in `python/stapes/scenarios.py`: a name, a description of what it
shows, and whether a stop is expected. The Python tests then run it and check that outcome, and
the viewer lists it. Nothing else changes.

== Rules that keep runs reproducible

- Never read the wall clock inside the simulation. Time is the tick count.
- Never use `std::normal_distribution` or any other standard distribution. Their output differs
  between standard libraries. Use `base::Rng`.
- Give each new noise source its own random stream, so it does not change the noise an existing
  sensor sees.
- Do not iterate over an unordered container where the order affects a result.
- Do not allocate in a tick. Use fixed-size Eigen types and `std::array`.
