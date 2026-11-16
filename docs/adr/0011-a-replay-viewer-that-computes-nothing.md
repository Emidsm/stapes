# ADR 0011: A browser viewer that replays and computes nothing

**Status:** accepted, 2026-11-15

## Context

The quickest evidence for a reviewer is a link that opens and shows the robot stopping when a
fault is injected. A viewer could be built in two ways: compile the core to WebAssembly and run
the simulation in the page, or record runs and replay them.

## Decision

- **Replay recorded runs.** The page loads JSON files written by the Python layer and draws them.
  A live WebAssembly build stays a stretch goal: it would be a second build of the core to keep
  deterministic and tested, for a demonstration that a recording already gives.
- **No kinematics in JavaScript.** Each frame carries the positions of the joints and the tool
  tip, computed by the core. The page joins them with cylinders. A second implementation of the
  kinematics, in another language and untested, could disagree with the first, and then the
  picture would be wrong while every test passed.
- **Two scales, shown separately.** The arm is 0.7 m long and the errors that matter are 0.1 mm,
  a ratio of 7000. No single 3D view can show both. So the arm is drawn at true scale, and the
  error is drawn on its own: a view down the drill axis with the 0.2 mm target and the 0.3 mm
  corridor as rings, and the error over time.
- **Red means one thing.** A safety stop or an injected fault, and nothing else, as in the plots
  and the manual. The tool turns red only when the monitor has stopped the robot.
- **Traces are generated, not committed.** The Pages workflow records the runs from the commit it
  deploys, so the page cannot show an older version of the code than the one beside it.
- **three.js is vendored at a pinned version**, with its licence, so the page works with no
  network access to a CDN and does not change when a CDN does.
- **No build step.** Plain ES modules, three files. Nothing to install to read or change it.

## Consequences

- The page needs an HTTP server, because browsers do not let a page read files from disk:
  `python3 -m http.server -d viewer`.
- The error series keep every control period; arm poses are stored every 10 ms and interpolated.
  Interpolating positions between two poses 10 ms apart is not exact, and it is used only for the
  picture, never for a number.
- A run with the monitor disabled leaves the scale of the across view by a factor of thousands.
  The view holds the marker at its edge and prints the distance, and the time chart switches to a
  logarithmic axis.
