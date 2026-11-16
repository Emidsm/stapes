# ADR 0013: A hands-on mode, driven from a gamepad

**Status:** accepted, 2026-11-16

## Context

The recorded runs show the monitor stopping the arm, but every run is scripted. In a
demonstration, someone holding a gamepad can try to break the system and watch the monitor
stop them. That is a stronger argument that the monitor is independent of whatever commands
the arm.

It is not how the robot this project is framed after is used. There, the trajectory is planned
from a CT scan, the robot drills it, and the surgeon supervises. A gamepad mode is a
demonstration of the monitor, and is described as one.

## Decision

- **A second simulation class with an operator in place of the plan.** `sim::JogSimulation`
  assembles the same arm, fault injector, controller and monitor as `KeyholeSimulation`, in the
  same tick order. Only the source of the target changes. It has four modes:
  - **Free.** The sticks set a tip velocity in the world frame. The tool keeps its direction,
    and each tick is solved by inverse kinematics from the previous target.
  - **Aligning.** On a press of A, a `task::JointMove` takes the arm onto the planned line at
    the 5 mm standoff, as the planned approach does.
  - **On the line.** The right stick sets only the feed along the line, up to the planned feed
    speed, between the standoff and the target. The left stick does nothing.
  - **Joint.** On a press of View, the operator turns one joint at a time, chosen with LB and
    RB, as on an industrial robot's teach pendant. Each joint runs at up to half its monitor
    speed limit and changes speed by at most 0.05 rad/s². A joint that is no longer chosen
    slows down at that rate rather than stopping dead. Near a limit, the joint slows down in
    time to stop 0.02 rad inside the monitor's own margin: a soft limit, so that turning a joint
    to its end is not a safety stop. The monitor still stops a joint that drives the tip into
    the bone off the line.
- **Nothing the operator does reaches the monitor.** The monitor sees the same sensors and
  limits as in a planned run. Jogging the tip into the bone off the line is allowed, and the
  monitor stops it inside the entry guard. A test does exactly that.
- **The operator's commands are limited so that the controller can follow them:**
  - The tip speed is capped at 25 mm/s.
  - The tip's acceleration is capped at 0.025 m/s². This value comes from measurement: at
    0.1 m/s², a stick pushed hard took the tracking error past the monitor's 1.8 mrad limit
    within 0.15 s. At 0.05 m/s², reversing the stick did the same once the gearboxes crossed
    their backlash.
  - The tip slows wherever a joint would pass half its speed limit, as it does near a
    singularity.

  Without these limits the monitor stopped the arm on ordinary stick movements. That was
  correct of the monitor, but useless in a demonstration.
- **Refusals instead of surprises.** Alignment, and switching between free and joint mode, are
  refused while the arm is moving, because both start from rest. It is also refused when the move would cut inside the entry guard off
  the line, the same check the planned approach makes before a run. Letting go of the line is
  refused anywhere but at the standoff. The page shows the reason.
- **Buttons act on the press.** The simulation keeps the previous input and acts when a button
  goes from released to pressed. That makes a run a function of the input sequence alone.
- **Faults start when the operator says.** `FaultInjector::start_at` moves a fault's start
  tick. The scenario's kind, joint and delay still apply.
- **A session is its inputs.** The Python server stores runs of equal input. Replaying a
  recording gives the same state hash, checked by `python -m stapes jog --replay`. A
  demonstration that went wrong can be replayed and examined like any other run.
- **The server uses the standard library only.** `http.server` serves the viewer and three JSON
  endpoints. The page sends the input once per animation frame, together with the number of
  control periods the browser clock has advanced, capped at 50. The simulation runs about five
  times faster than real time, so it keeps up. No new dependency is needed.
- **On Linux, the server reads the gamepad, not the browser.** Browsers map a controller they
  do not recognise by guesswork. A generic controller (USB ID `0079:181c`) under Firefox came
  through with its right stick as the triggers, a trigger fully pressed until first touched, and
  presses the page could not account for. The kernel reported the same controller exactly.
  `python/stapes/gamepad.py` reads the first joystick's evdev device with the standard library,
  handles both the generic HID and the xpad axis layouts, and hands the page the state in the
  browser's standard layout with every reply. Without Linux, or with `--browser-gamepad`, the
  page uses the browser's Gamepad API with the standard mapping.
- **The page maps, and decides nothing.** It turns the sticks into a velocity relative to the
  camera, keeps which joint is chosen and sends it with every input, so a recording holds it.
  In joint mode the right stick, which has no command there, turns the view instead, at
  up to 2.4 rad/s: the camera is the page's own and has no reason to be slow. The triggers are
  not used. The controller's Home button also sends a keyboard Home key, which
  the page ignores. The chosen joint and the link it turns are drawn amber: never red, which
  means a stop or a fault.
  The 3D view has a fixed height, and the panel's text is written only when it changes. A view
  sized by the panel was resized, and redrawn, whenever a button press changed a line of text:
  a visible flicker.
- **The planning helpers that both simulations need moved to `core/sim/src/planning.cpp`.** The
  recorded traces and the determinism hashes are byte-identical before and after the move.

## Consequences

- The hands-on mode needs the simulation running on the same machine. It is not on GitHub
  Pages, which serves files and cannot run the core. A WebAssembly build of the core would
  remove that limit. It is still the stretch goal from ADR 0011.
- Real time and the replay differ in one way: the browser decides how many periods each input
  lasts. The recording stores those counts, so the replay is still exact.
- The monitor is unchanged. Every limit in the hands-on mode comes from the same
  `SafetyConfig` as in the planned runs.
