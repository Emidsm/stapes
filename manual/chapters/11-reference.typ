#import "/template.typ": *

#chapter("Reference", summary: [Values and terms, in one place.])

== Timing

#data-table(
  (1fr, auto),
  [Quantity], [Value],
  [Control period, one tick], [1 ms],
  [Physics step], [0.1 ms],
  [Reference run], [3200 ticks, 3.2 s],
  [Reference tunnel], [10 mm],
  [Feed speed and acceleration], [5 mm/s, 25 mm/s²],
  [Longest delay the injector can represent], [255 ticks],
)

== Monitor limits of the reference configuration

#data-table(
  (1fr, auto),
  [Limit], [Value],
  [Corridor radius, for the tip and at the entry], [0.3 mm],
  [Overshoot allowed past the target], [0.3 mm],
  [Joint limit margin], [0.02 rad],
  [Link speed], [0.2 rad/s],
  [Tool force], [20 N],
  [Tracker against link encoders], [0.3 mm],
  [Motor encoder against link encoder], [1.0 mrad],
  [Link against target], [1.8 mrad],
  [Sample age], [2 ticks],
  [Debounce], [3 consecutive checks],
)

== Where things are

#data-table(
  (auto, 1fr),
  [Path], [Contents],
  [`core/base`], [Simulated clock, random generators, state hash],
  [`core/math`], [Rigid transforms and rotation error],
  [`core/kinematics`], [Arm model, forward kinematics, Jacobian, inverse kinematics],
  [`core/dynamics`], [Mass properties, inverse and forward dynamics, mass matrix],
  [`core/hardware`], [The two interfaces between software and robot],
  [`core/plant`], [The simulated robot],
  [`core/control`], [PID and the joint controller],
  [`core/safety`], [The safety monitor],
  [`core/faults`], [The fault injector],
  [`core/task`], [The keyhole trajectory and constraint],
  [`core/sim`], [A complete run, and the trace recorder],
  [`bindings`], [The Python module],
  [`python`], [Scenarios, plots, viewer export, and their tests],
  [`viewer`], [The replay page],
  [`tests`], [Unit, property, integration and determinism tests],
  [`tools`], [Traceability, test report, injected bugs],
  [`docs`], [Requirements, architecture, risk analysis, decision records],
)

== Glossary

/ Backlash: The small dead zone in a gearbox in which the teeth do not touch.
/ Corridor: The region around the planned line that the monitor allows the tool to be in.
/ Debounce: Requiring a condition to hold for several consecutive checks before acting on it.
/ Fulcrum: The fixed point a tool pivots about. Also called a remote centre of motion.
/ Keyhole: A narrow straight tunnel, and the constraint of working through one.
/ Latched: Said of a stop that stays in force until it is deliberately reset.
/ Link encoder: An encoder on the output of a gearbox, which measures the link itself.
/ Motor encoder: An encoder on the motor, before the gearbox. It cannot see the gearbox flex.
/ Plant: In control engineering, the thing being controlled. Here, the simulated robot.
/ Safety channel: The sensors and the brakes that the monitor uses and the controller does not.
/ Seed: The number that fixes all the randomness of a run.
/ Tick: One control period, 1 ms.
/ Tracker: An optical system that measures the tool's pose from outside the arm.
/ True state: What the simulator knows and no running code can see. Results are measured on it.
