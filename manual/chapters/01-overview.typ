#import "/template.typ": *

#chapter("What Stapes is", summary: [
  A simulated robot arm, the software that controls it, and an independent monitor that stops it
  when something goes wrong, all of it reproducible to the last bit.
])

Stapes is a simulation environment for a robot arm with six joints. The arm drills a straight,
narrow tunnel along a planned line: a _keyhole_. The task is modelled on robotic cochlear
implantation, where a tunnel about 1.8 mm wide is drilled from behind the ear to the inner ear,
passing between two nerves @weber2017. An error of a few tenths of a millimetre matters there, and
that sets the scale of everything in this manual.

It is named after the stapes, the smallest bone in the human body, which sits at the entrance of
the inner ear.

#note[
  Stapes is an engineering project and not a medical device. No physical robot is connected. The
  arm's geometry, the anatomy and the loads are illustrative, and nothing here is clinical data.
]

== What it contains

- *A simulated robot.* Rigid-body dynamics, motors with friction, gearboxes that flex and have
  backlash, and sensors with noise, quantisation and drift.
- *Control software* that sees the robot only through a narrow hardware interface, exactly as it
  would see a real one.
- *A safety monitor* that is independent of the controller: its own sensors, no shared state, and
  its own way to stop the robot.
- *A fault injector* that freezes sensors, sticks actuators and delays communication, without
  either side knowing it is there.
- *Verification tooling.* Every requirement is traced to the tests that verify it, and the tests
  themselves are tested by injecting bugs.

== The numbers that matter

#data-table(
  (1fr, auto, auto),
  [Measured on the true state of the simulated robot], [Value], [Required],
  [Tip to the planned line with no fault, worst of 20 seeds],
  [#measured("nominal_max_radial_um") µm], [< 200 µm],
  [Tip to the planned line under a 2 N drilling thrust],
  [#measured("thrust_2n_max_radial_um") µm], [< 200 µm],
  [Worst tip excursion with a frozen motor encoder, 24 runs],
  [#measured("frozen_motor_encoder_worst_excursion_um") µm], [< 500 µm],
  [Worst tip excursion with a stuck actuator, 24 runs],
  [#measured("stuck_actuator_worst_excursion_um") µm], [< 500 µm],
  [The same frozen encoder with the monitor disabled],
  [#measured("unmonitored_frozen_encoder_excursion_mm") mm], [> 50 mm],
  [Tip motion after a safety stop], [#measured("motion_after_stop_um") µm], [< 200 µm],
)

These values were read from the test results when this manual was built, using #data.compiler.
They are not typed by hand. The 0.2 mm figure is a design target taken from published work
@weber2017 @bell2013, not a claim about any real system.

== How to read this manual

@build gets the code built and tested. Chapters 3 to 6 explain the four parts of the system, in
the order a signal travels: the robot, the controller, the monitor, and the faults used to test
them. Chapters 7 and 8 are about running scenarios and looking at them. @verification explains
how the claims above are checked, and @extending how to add to the project without weakening
that.
