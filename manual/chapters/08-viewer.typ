#import "/template.typ": *

#chapter("The viewer", summary: [
  A page that replays recorded runs. It computes nothing, so it cannot disagree with the tests.
])

== Opening it

The viewer is a static web page. Record the runs, then serve the directory:

```sh
python -m stapes export viewer/traces
python3 -m http.server 8765 -d viewer
```

and open `http://127.0.0.1:8765`. It has to be served over HTTP, because a browser does not let
a page read files from disk. The published copy is rebuilt from every commit, with runs recorded
from that commit.

#figure(
  image("/build/figures/viewer.png", width: 100%),
  caption: [The viewer, 0.3 s after a motor encoder froze. The monitor has stopped the robot.],
)

== The three views

*The arm,* at true scale. Drag to orbit and scroll to zoom. The pale block is an illustrative
piece of bone around the planned tunnel; only the tunnel's position and direction come from the
plan.

*Down the drill axis.* The arm is 0.7 m long and the errors that matter are 0.1 mm, a ratio of
7000, so no single picture can show both. This view looks along the tunnel and shows where the
true tip is in the plane across it. The dotted ring is the 0.2 mm design target and the solid
ring is the monitor's 0.3 mm corridor.

*Error over time.* The distance of the true tip from the planned line through the run. The
vertical line is the moment being shown. When a run leaves the scale by orders of magnitude, as
the unmonitored one does, the axis becomes logarithmic.

== What red means

#stop-note[
  Red means a safety stop or an injected fault, and nothing else. A dashed red line marks the
  tick a fault begins. A solid red line marks the tick the monitor stops the robot, and from
  then on the tool and the tip marker are red.
]

== Controls

#data-table(
  (auto, 1fr),
  [Control], [Effect],
  [Scenario], [Loads another recorded run. The address changes too, so a link can point at one scenario.],
  [Play, or Space], [Plays and pauses.],
  [Speed], [Playback speed. A stop is decided within a few milliseconds, so 0.1× is useful.],
  [Slider, or the arrow keys], [Moves through the run. The arrow keys step by 10 ms.],
)

== What it does not do

The viewer contains no simulation and no kinematics. Each frame of a recording carries the
positions of the joints and the tool tip as the C++ core computed them, and the page joins them
up. Arm poses are stored every 10 ms and interpolated for smooth motion; the error series keep
every control period. Interpolation is used for the picture only, never for a number.
