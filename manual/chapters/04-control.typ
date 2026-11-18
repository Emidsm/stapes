#import "/template.typ": *

#chapter("Planning and control", summary: [
  From a planned line in space to motor torques, a thousand times a second.
])

== The keyhole constraint

Drilling through a keyhole means two things at once: the tip follows the planned line, and the
tool's shaft passes through the entry point. Both are captured by one number, the distance from
a fixed point to the line through the tool's axis.

The same number describes a second scenario. In laparoscopy a tool pivots about the point where
it enters the body, a _remote centre of motion_. That is the same constraint with the tip moving
across the axis instead of along it. Stapes tests both: under closed-loop control the tool axis
stays within #measured("rcm_max_axis_offset_um") µm of a fixed fulcrum while tilting by
#measured("rcm_largest_tilt_mrad") mrad.

== The plan

A plan is an entry point, a target point, and a feed: accelerate, hold the feed speed, slow down.
The reference plan is a tunnel 10 mm long, fed at 5 mm/s. It is tilted out of the arm's plane on
purpose, so that every joint has to move. A joint that stands still could hide a fault.

Before anything moves, the whole tunnel is walked in steps of 0.5 mm, solving inverse kinematics
at each. A plan that cannot be reached is refused with the arm unmoved.

== One control period

Every millisecond:

+ The feed profile gives the depth the tip should have reached.
+ Inverse kinematics turns the pose at that depth into six joint angles, starting from the
  previous solution. The solver is Levenberg-Marquardt with joint limits as bounds.
+ The joint controller turns the joint angles and the motor encoder readings into torques.

== The joint controller

Each joint has a PID controller with a filtered derivative and anti-windup. Two model-based
terms are added:

- *Gravity compensation.* The torque needed to hold the arm against gravity in its current pose
  is computed from the dynamic model and fed forward, so the PID only has to correct errors.
- *Gearbox compensation.* The controller predicts how far each gearbox flexes under the gravity
  load and aims the motor past the target by that amount. Without it the tip sags by
  #measured("tip_error_without_compensation_um") µm; with it, by
  #measured("tip_error_with_compensation_um", digits: 2) µm.

The controller uses its own copy of the model and its own estimates of the gearboxes. A test
gives it a model that is wrong by 5 % in mass and 10 % in stiffness, and the tip stays within
tolerance.

#why[
  The gains were chosen for stability under delay, not for the smallest error. The first gains
  were more accurate, and one millisecond of communication delay made them diverge. Every joint
  now has the same 200 rad/s crossover, which stays within 0.2 mm with five milliseconds of delay
  in each direction. ADR 0008 records the measurements.
]

== What the controller cannot do

The controller steers by motor encoders, so it corrects for gearbox flex only as far as its
model predicts it. A drilling thrust is not in its model. Up to 2 N the tip stays within the
0.2 mm target, at #measured("thrust_2n_max_radial_um") µm. Beyond that it does not, and it is
the monitor, which reads the tracker, that notices. Using the tracker to correct the tip in
closed loop would be the next step for accuracy under load. It is not done, and ADR 0009 says so.
