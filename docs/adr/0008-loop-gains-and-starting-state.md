# ADR 0008: Loop gains from delay margin, and a loaded starting state

**Status:** accepted, 2026-11-04

## Context

The first fault runs of M2 exposed two defects in M1, which its tests could not have shown because
no test delayed anything or looked at the first moments of a run.

**Fragile gains.** One tick of delay on readings and commands made the loop diverge, taking the
tip 21 mm off the line. The M1 gains put the crossover of joints 1 and 2 at
`kd / J = 100 / 0.15 = 667 rad/s`, too high for a 1 kHz loop to have any margin against delay.

**An unloaded start.** The tool left the line by 256 um in the first moments of every run. The
plant started each motor at the same angle as its link, with the gearboxes carrying no load, and
the arm dropped until they took up gravity. A real arm starts on its brakes with its gearboxes
already wound up.

## Decision

- Set every joint to the same 200 rad/s crossover: `kd = 200 J`, `kp = 30 kd`, `ki = 5 kp`.
- Start the motors where the gearboxes carry gravity: at `q + load / stiffness`, with the teeth in
  contact on the loaded side.
- Do not add velocity feedforward. It was implemented and measured, and then removed, as below.

## Evidence

Measured on the tilted reference tunnel on 2026-11-04. Each cell is the worst true distance of the
tip from the line, in micrometres, with that many ticks of delay on readings and on commands.

| Crossover, `kp/kd`, `ki/kp` | 0 | 1 | 2 | 3 | 5 | 10 |
|---|---|---|---|---|---|---|
| M1 gains | 55 | 21445 | 39158 | 23866 | 8062 | 34768 |
| 150 rad/s, 20, 5 | 108 | 108 | 113 | 123 | 166 | 257 |
| 200 rad/s, 20, 5 | 85 | 88 | 94 | 104 | 175 | 28357 |
| **200 rad/s, 30, 5 (chosen)** | **70** | **71** | **72** | **76** | **113** | 39111 |
| 200 rad/s, 20, 10 | 75 | 76 | 77 | 95 | 121 | 30906 |
| 250 rad/s, 30, 10 | 55 | 58 | 57 | 88 | 7000 | 36721 |
| 300 rad/s, 40, 10 | 46 | 50 | 65 | 101 | 8781 | 77232 |

The chosen row is the most accurate one that survives 5 ticks, more than twice the 2 ticks the
monitor allows.

Velocity feedforward cut the peak joint tracking error from 1.14 to 0.71 mrad, but did not improve
what matters: the worst tip error stayed at 70 um with or without it. Removing it and widening the
monitor's tracking limit gave the same fault campaign result, 439 um against 432 um with no false
stops, for one parameter fewer through the whole control path.

## Consequences

- Peak accuracy is lower than the highest gains could give, and stability is the better trade:
  the chosen gains are within the 0.2 mm design target up to 5 ticks of delay.
- Only the 150 rad/s row survives 10 ticks. The monitor stops the robot long before that, so
  giving up 38 um of accuracy to reach it would buy nothing.
- The 0.2 mm figure holds for a feed of 5 mm/s, with no load on the tool. See ADR 0009 for what a
  thrust does to it.
