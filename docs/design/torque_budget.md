# Torque budget and sizing

> **Generated file. Do not edit by hand.** Regenerate with `python tools/torque_budget.py`.
> Source: `cad/params/arm_params.json` (design revision `0.3-phase1`).
> Printed-part masses: **CAD-derived** (`cad/export/masses.json`).

## 1. Result

- **Reach:** 255 mm (J1 axis to grip centre, arm horizontal). Fingertips reach 270 mm.
- **Links:** upper arm 80 mm, forearm 80 mm, wrist axis to grip centre 95 mm.
- **Rated payload:** 50 g at full horizontal reach.
- **Moving mass beyond the shoulder:** 292 g with pots, 278 g without (plus payload).
- **Verdict:** PASS: safety factor >= 2 on every joint.

## 2. Joint table

Rule (spec 4.3): required torque <= rated stall torque at 6 V / 2. "Available" is that derated value. Pots are assumed fitted (heavier, worst case).

| Joint | Servo | Load case | Required (kgf·cm) | Rated @6 V | Available (derated) | Margin | **SF** | Pass |
|---|---|---|---|---|---|---|---|---|
| J1 base_yaw | MG996R | inertia at default accel + friction | 0.79 | 11.0 | 5.50 | +86 % | **14.00** | yes |
| J2 shoulder_pitch | MG996R | static gravity, arm horizontal | 4.91 | 11.0 | 5.50 | +11 % | **2.24** | yes |
| J3 elbow_pitch | MG996R | static gravity, arm horizontal | 2.37 | 11.0 | 5.50 | +57 % | **4.64** | yes |
| J4 wrist_pitch | MG90S | static gravity, arm horizontal | 0.85 | 2.2 | 1.10 | +23 % | **2.59** | yes |
| J5 wrist_roll | MG90S | off-axis gravity + friction + inertia | 0.33 | 2.2 | 1.10 | +70 % | **6.63** | yes |
| J6 gripper | MG90S | squeeze at 4 cm jaw lever | gives ~2.7 N squeeze | | | | holds ~330 g by friction | yes |

### What the margin really means (read this)

The SF above is measured against the **datasheet** torque, as the spec asks. Two things eat into it in real life:

| Joint | SF vs datasheet @6 V | SF with rail sagging to 5.4 V (low battery) | SF if the servo is a weak clone (60 % of rated) |
|---|---|---|---|
| J1 | 14.00 | 12.98 | 8.40 |
| J2 | 2.24 | 2.08 | 1.34 |
| J3 | 4.64 | 4.30 | 2.78 |
| J4 | 2.59 | 2.35 | 1.55 |
| J5 | 6.63 | 6.03 | 3.98 |

- Even with weak clones every joint stays above 1.0, so the arm still holds the rated payload at full reach; it just has little reserve there. Closer in, the load drops quickly.
- Accelerating the shoulder at the default limit (120 °/s²) adds only 0.19 kgf·cm at J2, so statics dominate and the gentle default limits are appropriate.
- A stalled hobby servo overheats. SF 2 means the shoulder servo works at about half its stall current when holding the worst-case pose: fine for minutes, not for hours. Park the arm when idle.

## 3. Where the shoulder torque comes from

Arm horizontal. Position is measured from the shoulder (J2) axis. 1 kgf·cm = 10 000 g·mm.

| Item | Mass (g) | Position (mm) | J2 torque (kgf·cm) | Source |
|---|---|---|---|---|
| upper arm (printed fork + elbow servo cradle) | 35.6 | 45 | 0.159 | CAD |
| J3 servo MG996R (in upper arm, at elbow) | 55.0 | 72 | 0.396 | servo:J3 |
| J3 metal horn + horn screws | 6.0 | 80 | 0.048 | catalogue |
| J3 passive bearing 608ZZ + pivot pin + M3 screw | 15.0 | 80 | 0.120 | catalogue |
| J3 pot | 7.0 | 80 | 0.056 | catalogue |
| J3 fasteners (M3 screws + nuts) | 7.0 | 80 | 0.056 | estimate |
| cables along upper arm (4 servo + 1 pot lead) | 8.0 | 40 | 0.032 | estimate |
| forearm (printed fork + beam + wrist bearing stub) | 31.1 | 119 | 0.369 | CAD |
| cables along forearm | 5.0 | 120 | 0.060 | estimate |
| J4 servo MG90S (at wrist) | 13.4 | 160 | 0.214 | servo:J4 |
| J4 horn + passive bearing 608ZZ + pivot pin | 16.0 | 160 | 0.256 | catalogue |
| J4 pot | 7.0 | 160 | 0.112 | catalogue |
| J4 fasteners | 3.0 | 160 | 0.048 | estimate |
| wrist block (printed, holds roll servo) | 15.4 | 172 | 0.265 | CAD |
| J5 servo MG90S (axial) | 13.4 | 185 | 0.248 | servo:J5 |
| J5 horn + support ring | 3.0 | 200 | 0.060 | estimate |
| gripper body + fixed jaw (printed) | 17.4 | 215 | 0.374 | CAD |
| gripper fingers, pinion, pads (printed) | 12.7 | 251 | 0.319 | CAD |
| J6 servo MG90S (gripper) | 13.4 | 212 | 0.284 | servo:J6 |
| hand fasteners + J6 horn | 5.0 | 205 | 0.102 | estimate |
| cables in hand | 3.0 | 185 | 0.056 | estimate |
| **payload** | 50 | 255 | 1.275 | design target |
| **Total** | 342 | | **4.91** | |

The payload is 26 % of the shoulder load; the arm's own weight is the rest. Items with source `CAD` use the real part volumes; `estimate-print` items are hand estimates with ~15 % contingency.

## 4. Base tipping check (preliminary)

Arm horizontal with payload, pointing straight at the nearest tipping line (73 mm from the J1 axis: 170 mm base, feet inset 12 mm). Stationary mass in the base: 978 g.

- Restoring moment: 7.3 kgf·cm
- Overturning moment: 2.5 kgf·cm
- **Tipping safety factor: 2.9** (target >= 1.5 static; sudden stops add load, hence the margin)

## 5. Sizing sweep: longest arm for each payload

Equal upper-arm and forearm length, wrist-to-grip fixed at 95 mm (set by packaging), J1/J2/J3 = MG996R. Links shorter than 75 mm do not physically fit the joints.

| Wrist (J4) servo | Payload (g) | Max link length (mm) | Max reach (mm) | SF J2 | SF J4 | Note |
|---|---|---|---|---|---|---|
| MG90S | 50 | 92 | 279 | 2.01 | 2.58 | limited by J2 |
| MG90S | 75 | 80 | 255 | 2.00 | 2.02 | limited by J2 |
| MG90S | 100 | n/a | n/a | | | not possible: J4 SF 1.66 even at 75 mm links |
| MG90S | 125 | n/a | n/a | | | not possible: J4 SF 1.41 even at 75 mm links |
| MG90S | 150 | n/a | n/a | | | not possible: J4 SF 1.22 even at 75 mm links |
| S3003 | 50 | 82 | 259 | 2.01 | 4.81 | limited by J2 |
| S3003 | 75 | n/a | n/a | | | not possible: J2 SF 1.93 even at 75 mm links |
| S3003 | 100 | n/a | n/a | | | not possible: J2 SF 1.75 even at 75 mm links |
| S3003 | 125 | n/a | n/a | | | not possible: J2 SF 1.59 even at 75 mm links |
| S3003 | 150 | n/a | n/a | | | not possible: J2 SF 1.46 even at 75 mm links |

The sweep shows the *limit* (SF exactly at the minimum) and always uses the Phase 0 hand estimates, because CAD masses only exist for the link lengths that were actually modelled.

## 6. Units

1 kgf·cm = 0.0981 N·m. Shoulder requirement: 4.91 kgf·cm = 0.481 N·m.
