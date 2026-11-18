#import "/template.typ": *

#chapter("The simulated robot", summary: [
  A robot whose controller cannot see everything that is true about it, because real robots are
  like that.
])

== The arm

The reference arm has six revolute joints: base yaw, shoulder pitch, elbow pitch, forearm roll,
wrist pitch and tool roll. The upper arm and the forearm are 0.30 m each, the last three joint
axes meet at one point to form a spherical wrist, and the tool is 0.10 m long. The geometry is
described the way URDF describes it: for each joint, where it sits relative to its parent and
the axis it turns about.

#data-table(
  (auto, 1fr, auto, auto),
  [Joint], [Motion], [Range], [Torque limit],
  [1], [Base yaw], [±170°], [60 N m],
  [2], [Shoulder pitch], [±120°], [60 N m],
  [3], [Elbow pitch], [−80° to 140°], [30 N m],
  [4], [Forearm roll], [±170°], [8 N m],
  [5], [Wrist pitch], [±120°], [8 N m],
  [6], [Tool roll], [±175°], [4 N m],
)

== Dynamics

The arm obeys rigid-body dynamics, not only kinematics. Inverse dynamics use the recursive
Newton-Euler algorithm, and the mass matrix uses the composite rigid body algorithm. Both are
checked against hand calculations and against each other. The physics advances in steps of
0.1 ms, ten for each control period of 1 ms, using semi-implicit Euler integration.

#why[
  A gearbox is a very stiff spring. Explicit Euler integration steadily adds energy to a stiff
  oscillator until it diverges. Semi-implicit Euler updates velocity first and then position
  from the new velocity, which keeps the energy bounded. A test measures the energy drift in free
  motion and checks that it vanishes at the integrator's order as the step shrinks.
]

== Each joint is two masses

A joint is not one rigid thing. The motor is one mass and the link is another, and the gearbox
between them is elastic and has backlash: a small dead zone in which the teeth do not touch.
Motor friction combines Coulomb friction, a Stribeck peak at low speed, and viscous friction.

The motor encoders sit on the _motor_ side of the gearbox, as on a real geared arm. So when a
load bends a gearbox, the link moves and the encoder reading does not change.

This one modelling choice is what gives the rest of the project its meaning. It opens a gap
between what the controller believes and what is true, and that gap is exactly what the safety
channel is there to see.

== Sensors, with their defects

#data-table(
  (auto, 1fr, auto),
  [Sensor], [Model], [Read by],
  [Motor encoders], [20 bits per turn, Gaussian noise, quantised to whole counts], [Controller, monitor],
  [Link encoders], [19 bits per turn, on the output of each gearbox], [Monitor only],
  [Optical tracker], [Tool pose from outside the arm; 30 µm position noise per axis, 100 µrad orientation noise], [Monitor only],
  [Force sensor], [Six axes; noise, scale error, and a bias that wanders as a random walk], [Controller, monitor],
)

The link encoders and the tracker form the _safety channel_. They see what the motor encoders
cannot: the link encoders see past the gearbox, and the tracker sees the tool itself. The force
sensor is shared by both channels, because the robot carries only one. That is a known weakness,
recorded in the risk analysis.

== Brakes

The safety channel has its own actuator: the brakes. Engaging them removes motor torque, makes
later torque commands have no effect, and applies a holding torque to each motor, limited to its
rating. The links can still move as far as the gearboxes flex, which is why a stopped tool is
not perfectly still: it settles within #measured("motion_after_stop_um") µm of where it stopped.

== The hardware boundary

Control software reaches the robot through two small interfaces and nothing else:

```cpp
class RobotInterface {
  virtual SensorSample read_sensors() const = 0;
  virtual void command_torques(const JointVector& torques_nm) = 0;
};

class SafetyInterface {
  virtual SafetySample read_safety_sensors() const = 0;
  virtual void engage_brakes() = 0;
  virtual void release_brakes() = 0;
};
```

The simulated arm implements both. A real robot would implement the same two, and the software
on the other side would not change. The controller and the monitor are separate libraries that
do not link the simulator, so they cannot read its true state even by mistake: the build would
fail.
