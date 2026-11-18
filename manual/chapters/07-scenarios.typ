#import "/template.typ": *

#chapter("Running scenarios", summary: [
  Nine named runs, each showing one thing.
])

The Python layer names a set of scenarios and runs them. It adds no behaviour: every scenario is
the reference drilling task with at most one thing going wrong, and everything that happens in a
run happens in the C++ core.

== The scenarios

```sh
python -m stapes list
```

#data-table(
  (auto, 1fr, auto),
  [Name], [What it shows], [Ends in],
  [`nominal`], [The reference tunnel with no fault.], [no stop],
  [`thrust-2n`], [A drilling thrust the control loop can hold.], [no stop],
  [`thrust-5n`], [More thrust than it can hold: the tracker sees the tip leave the line.], [stop],
  [`frozen-motor-encoder`], [The controller is blind on one joint; the link encoder is not.], [stop],
  [`frozen-motor-encoder-unmonitored`], [The same fault with the monitor's limits out of reach.], [no stop],
  [`frozen-link-encoder`], [A sensor of the safety channel itself freezes.], [stop],
  [`frozen-tracker`], [The tracker repeats its last pose while the arm moves on.], [stop],
  [`stuck-actuator`], [One motor ignores its commands.], [stop],
  [`comms-delay`], [Readings and commands arrive 20 ms late.], [stop],
)

A fault always begins at tick 800, which is 0.8 s into the run, with the feed under way.

== Running one

```sh
python -m stapes run frozen-motor-encoder --seed 1
```

```
scenario        frozen-motor-encoder (seed 1)
max tip error   158.4 um from the planned line
max axis offset 157.0 um at the entry
final depth     3.755 mm
monitor         stopped at tick 830: encoder_disagreement
detection       30 ms after the fault began
```

The same seed gives the same output, to the last digit, every time. A different seed changes the
sensor noise and so the exact numbers. The output above is an example from one compiler;
the values in the tables of this manual are the ones read from the test results.

== Reading a result

#plot("radial-nominal", [
  A healthy run. The tip stays well inside the 0.2 mm design target while the tool feeds 10 mm
  and settles.
])

#plot("radial-thrust-5n", [
  A 5 N thrust bends the gearboxes, and the tip leaves the line. The tracker sees it and the
  monitor stops the robot. With no injected fault there is no dashed line, only the stop.
])

#plot("radial-stuck-actuator", [
  A stuck actuator is the slowest fault to detect: the joint drifts off its target gradually.
])

== From Python

```python
import stapes

trace = stapes.run(stapes.SCENARIOS["stuck-actuator"], seed=1)
print(trace.cause_name, trace.detection_ticks)
print(trace.radial_m.max())          # one value per control period
```

A `Trace` holds one row per control period: the true joint angles, the targets, the tip
position, the tool axis, the distances from the plan, and whether a fault was active or the
brakes engaged. These are the simulator's true values, which no running code can see. They are
for analysis only.

For a run that is not one of the named scenarios, drive the simulation directly:

```python
scenario = stapes.KeyholeScenario.reference(seed=7)
scenario.fault = stapes.Fault(stapes.FaultKind.frozen_tracker, start_tick=400)
sim = stapes.KeyholeSimulation(scenario)
while sim.stop_tick is None and sim.ticks < 3200:
    sim.step()
```

== Figures and viewer files

```sh
python -m stapes plots build/figures
python -m stapes export viewer/traces
```

The first writes the figures used in this manual. The second writes one JSON file per scenario
for the viewer described in the next chapter.
