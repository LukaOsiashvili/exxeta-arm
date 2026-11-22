# ADR-0006: Rack-and-pinion parallel gripper

- **Status:** accepted (owner request, 2026-11-15). Supersedes the claw gripper in ADR-0002 §2.
- **Spec:** §4.4 (gripper type chosen by the design, opening ≥ 50 mm, swappable fingertips)

## Decision

A **parallel gripper driven by one MG90S**:

- The J6 servo stands upright on the hand axis. A printed **pinion** (18 teeth, module 1, pitch radius 9 mm) sits on its output boss and is driven by the servo horn.
- **Two identical racks**, one on each side of the pinion, move in opposite directions. Each carries one finger.
- The racks slide on shelves of the gripper body. A tongue on the back of each rack runs in a groove, so the fingers cannot lift out.
- Fingertips are push-on sleeves (TPU, or PETG with rubber tape), as before.

| | Value |
|---|---|
| Opening | 0 to 50 mm (160° of servo travel; 0.157 mm of finger travel per degree) |
| Grip force at SF 2 | about 6 N per finger (1.1 kgf·cm ÷ 0.9 cm ÷ 2 fingers) |
| Holds by friction (μ ≈ 0.6, two faces) | about 700 g, far above the 50 g payload |
| Printed mass | body 15 g, two fingers 9 g, pinion under 1 g, pads 4 g |

## Why this is better than the claw

- **The grip centre stays on the hand axis** for every object size. With the claw one jaw was fixed, so the object's centre moved sideways as the gripper opened. That matters for IK: the software can now aim the hand at the object's centre.
- **Parallel faces** hold boxes and cylinders without pushing them out.
- **About twice the grip force** at the same servo load, because the pinion radius (9 mm) is much shorter than the claw's 40 mm lever.

## Costs

- Roughly 4 g more than the claw, and more sliding surfaces. The sizing gate still passes (shoulder SF 2.24, wrist SF 2.59).
- When the gripper is open, the tails of the racks stick out sideways from the gripper body by up to 11 mm.
- Printed module-1 teeth need a well-tuned printer (0.1–0.15 mm layers for the pinion). Backlash of about 0.3 mm at the fingers is expected and harmless.
- The J6 horn (and the J5 horn under the servo) must be cut to 12 mm.

## Software note

The firmware and the planner still command the **servo angle** of J6 (0° closed, 160° open). The conversion to finger opening is stored in `description/robot_info.json` (`gripper_m_per_rad`); the URDF models the fingers as two prismatic joints, the second one mimicking the first.
