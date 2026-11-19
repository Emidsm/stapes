# Risk analysis

This analysis is **structured after** ISO 14971, the standard for risk management of medical
devices. It does not claim compliance with it. Stapes is an engineering project and not a medical
device: there is no patient, no clinical data, and the severities below are illustrative. The
purpose is to show the method, and to make each safety requirement answer to a reason.

## Method

1. **Identify hazards.** Each hazard is a way the simulated system could do harm if it were a real
   one: what goes wrong, what causes it, and what the harm would be.
2. **Estimate the risk** with no control in place, as a severity and a probability.
3. **Control it** with requirements. A control is always a requirement ID from
   [`requirements.toml`](requirements/requirements.toml), never a sentence, so a control can be
   traced to the tests that verify it.
4. **Verify the controls.** `tools/traceability.py --hazards` fails if a hazard has no control, if a
   control is not a known requirement, or if the tests of a control did not pass. A control that
   is not verified does not count.
5. **State the residual risk.** What the controls do not cover is written down, not left out.

The hazards themselves are in [`risk/hazards.toml`](risk/hazards.toml), which is the single source
for the table below and for the check in step 4.

## Scales

| Severity | Meaning |
|---|---|
| 5 | Permanent injury |
| 4 | Injury that needs intervention |
| 3 | Minor injury, or damage to the device |
| 2 | The procedure is interrupted |
| 1 | Inconvenience |

| Probability | Meaning |
|---|---|
| 5 | Expected in most procedures |
| 4 | Expected in some procedures |
| 3 | Could happen over the life of a device |
| 2 | Unlikely, but has a known cause |
| 1 | Not expected |

Risk is acceptable here when probability is 1, or when it is 2 and severity is at most 4. Anything
else needs a control, or an explicit decision recorded under "Residual risks" below.

## Hazards and controls

| ID | Hazard | Sev. | Prob. | Controls | Residual prob. |
|---|---|---|---|---|---|
| HAZ-01 | The drill tip leaves the planned tunnel sideways | 5 | 3 | SRS-KEY-006, SRS-KEY-007, SRS-SAF-008, SRS-SAF-021 | 1 |
| HAZ-02 | The drill goes deeper than the planned target | 5 | 2 | SRS-KEY-004, SRS-KEY-006, SRS-SAF-008 | 1 |
| HAZ-03 | The tool tilts and its shaft presses on the keyhole | 3 | 3 | SRS-KEY-001, SRS-KEY-002, SRS-SAF-008 | 1 |
| HAZ-04 | A motor encoder freezes with a fresh timestamp | 5 | 2 | SRS-SAF-010, SRS-SAF-019, SRS-SAF-023 | 1 |
| HAZ-05 | A sensor on the safety channel freezes | 4 | 2 | SRS-ARM-008, SRS-SAF-009, SRS-SAF-019 | 1 |
| HAZ-06 | An actuator holds its last torque | 4 | 2 | SRS-SAF-011, SRS-SAF-019 | 1 |
| HAZ-07 | Readings or commands arrive late | 5 | 3 | SRS-CTL-011, SRS-SAF-004, SRS-SAF-020 | 1 |
| HAZ-08 | A reading, target or command is not a number | 4 | 2 | SRS-ACT-003, SRS-CTL-004, SRS-KIN-005, SRS-SAF-003 | 1 |
| HAZ-09 | The tool presses on tissue with too much force | 4 | 3 | SRS-SAF-007, SRS-SAF-021 | 2 |
| HAZ-10 | A joint reaches its limit or moves too fast | 3 | 2 | SRS-KIN-004, SRS-SAF-005, SRS-SAF-006 | 1 |
| HAZ-11 | The robot keeps moving after a stop | 5 | 2 | SRS-ARM-009, SRS-FLT-007, SRS-SAF-013, SRS-SAF-022 | 1 |
| HAZ-12 | A restart while the cause is still present | 4 | 3 | SRS-SAF-014 | 1 |
| HAZ-13 | Motion starts on a plan that cannot be completed | 4 | 2 | SRS-KEY-005, SRS-KIN-005 | 1 |
| HAZ-14 | The monitor fails together with the controller | 5 | 3 | SRS-ARM-008, SRS-CTL-006, SRS-SAF-016 | 2 |
| HAZ-15 | The monitor stops a healthy procedure | 2 | 3 | SRS-SAF-002, SRS-SAF-012, SRS-SAF-018 | 1 |
| HAZ-16 | An incident cannot be reproduced afterwards | 3 | 3 | SRS-DET-007, SRS-RT-003, SRS-TRC-002, SRS-TRC-003 | 1 |
| HAZ-17 | The tool reaches the entry off the line at the end of the approach | 5 | 3 | SRS-KEY-010, SRS-KEY-011, SRS-SAF-024, SRS-SAF-025 | 1 |

## How the controls work together

Three ideas carry most of the table.

**A second channel.** The controller steers by motor encoders. The monitor has its own sensors:
link encoders after each gearbox and an optical tracker outside the arm. Comparing independent
sensors finds a frozen one whichever froze (HAZ-04, HAZ-05), and the build prevents the monitor
from reading the controller's state (HAZ-14).

**Outcomes, not detections.** A fault on a joint that is not moving may show nothing for a while.
So SRS-SAF-019 is not "every fault is detected": it is "the true tip stays within 0.5 mm of the
plan", measured on the simulator's true state, which no running code can read. The fault campaign
runs each fault on every joint, at two times and with two seeds.

**A baseline.** A campaign in which nothing bad happens would also pass if the faults were
harmless. SRS-SAF-023 runs the same faults with the monitor's limits out of reach and requires
the tip to leave by more than 50 mm. In the measured run it leaves by about a metre.

## Residual risks

These are known, accepted for a simulation, and would need work on a real system.

- **One force sensor (HAZ-05, HAZ-09).** The robot carries one, read by both channels. A frozen
  force reading cannot be found by comparison, because there is nothing to compare it with.
- **A window between target and corridor (HAZ-01).** The design target is 0.2 mm and the monitor's
  corridor is 0.3 mm. Under a thrust between 2 N and 5 N the tip can sit between the two without a
  stop. Closing it needs the tracker in the control loop, which is not done: see ADR 0009.
- **An optimistic tracker (HAZ-07).** The simulated tracker samples at 1 kHz with no latency. A
  real one is slower, so the detection times measured here are lower bounds.
- **One process (HAZ-14).** Independence is enforced by the build and by separate sensors. On a
  real system the monitor would also run on separate hardware.
- **Slow detection of a stuck actuator (HAZ-06).** The joint drifts gradually, so this fault takes
  the longest to cross a limit. The excursion stays within the harm limit, with less margin than
  the others.
- **The approach is checked only against the tunnel (HAZ-17).** The corridor starts 2 mm before
  the entry, and the move from rest is refused if it would cut inside that guard off the line.
  Collisions of the arm with anything else on the way are not modelled. Phase-dependent limits,
  which would allow a faster approach, are deliberately not used (ADR 0012).
- **Platform-bound reproducibility (HAZ-16).** A run is bit-exact for a given compiler and maths
  library, and CI checks GCC against Clang on one platform. Nothing more is claimed.
