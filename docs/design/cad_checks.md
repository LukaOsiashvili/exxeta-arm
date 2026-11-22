# CAD checks

> **Generated file.** Regenerate with `python tools/export_all.py`.
> Design revision `0.3-phase1`.

## Summary

- Print-bed fit (220 x 220 x 250 mm): **PASS**
- Interference check: **PASS**
- Sizing gate (torque budget on CAD masses): **PASS**

## Parts

| # | File | Qty | Mass each (g) | Bounding box (mm) | Fits bed | Print orientation | Supports |
|---|---|---|---|---|---|---|---|
| 01 | `01_base_shell_1x.stl` | 1 | 221.4 | 170.0 x 170.0 x 65.0 | yes | upside down (deck on the bed) | none |
| 02 | `02_base_panel_1x.stl` | 1 | 85.7 | 163.4 x 163.4 x 3.0 | yes | flat, outside face down | none |
| 03 | `03_j1_bracket_1x.stl` | 1 | 9.5 | 69.5 x 83.9 x 4.0 | yes | flat, top face up | none |
| 04 | `04_clamp_ring_1x.stl` | 1 | 16.9 | 112.0 x 112.0 x 10.0 | yes | upside down (flat top face on the bed) | none |
| 05 | `05_turntable_1x.stl` | 1 | 68.6 | 90.5 x 90.5 x 69.8 | yes | as modelled: disc flat on the bed, tower pointing up | none (45 degree groove, short bridges) |
| 06 | `06_upper_arm_1x.stl` | 1 | 35.6 | 26.2 x 65.2 x 110.4 | yes | lying on the flat back (-X) face | none; the servo pocket roof is a 41 mm bridge |
| 07 | `07_forearm_1x.stl` | 1 | 31.1 | 26.2 x 65.2 x 106.2 | yes | lying on the flat back (-X) face | none |
| 08 | `08_wrist_block_1x.stl` | 1 | 12.2 | 33.5 x 25.0 x 34.0 | yes | standing on the -Z face (J5 pocket opening up) | none |
| 09 | `09_wrist_cap_1x.stl` | 1 | 3.2 | 34.0 x 24.0 x 9.9 | yes | as modelled (frame on the bed, journal up) | a little, under the top plate inside the frame |
| 10 | `10_gripper_body_1x.stl` | 1 | 17.4 | 38.0 x 33.2 x 43.4 | yes | as modelled (skirt down) | under the base plate around the skirt |
| 11 | `11_gripper_finger_2x.stl` | 2 | 3.4 | 36.0 x 21.6 x 37.8 | yes | lying on its back (rack teeth pointing up) | a little under the finger |
| 21 | `21_gripper_pinion_1x.stl` | 1 | 0.9 | 20.0 x 19.8 x 4.7 | yes | flat, horn pocket up | none |
| 12 | `12_fingertip_2x.stl` | 2 | 2.5 | 9.2 x 16.4 x 29.2 | yes | open end up | none |
| 13 | `13_pivot_pin_3x.stl` | 3 | 0.6 | 10.0 x 10.8 x 11.5 | yes | head down | none |
| 14 | `14_pivot_cover_3x.stl` | 3 | 1.9 | 24.2 x 6.5 x 34.1 | yes | flat face down | none |
| 15 | `15_pot_pod_3x.stl` | 3 | 4.0 | 24.2 x 19.0 x 34.1 | yes | flange down | none |
| 16 | `16_pot_coupler_3x.stl` | 3 | 1.6 | 14.0 x 12.9 x 14.0 | yes | standing, pot end up | none |
| 17 | `17_j1_pot_gear_1x.stl` | 1 | 3.2 | 28.0 x 27.9 x 6.0 | yes | flat | none |
| 18 | `18_tolerance_test_1x.stl` | 1 | 55.6 | 122.0 x 62.0 x 9.0 | yes | flat | none |
| 19 | `19_test_rig_stand_1x.stl` | 1 | 57.2 | 73.1 x 90.0 x 79.3 | yes | lying on the -X face (same as the arm links) | none |
| 20 | `20_test_rig_arm_1x.stl` | 1 | 29.5 | 26.2 x 65.2 x 123.1 | yes | lying on the -X face | none |

Printed mass of the standard arm (no pots, no test parts): **522 g** (PETG, 85 % effective fill).

## Interference

No overlaps above 2.0 mm³ in: static fit of all parts and servos, the poses zero / home / reach / rest, and five samples across the range of every joint.

## Sizing gate output

```
wrote docs\design\torque_budget.md
  J2 MG996R  SF 2.24
  J3 MG996R  SF 4.64
  J4 MG90S   SF 2.59
  J5 MG90S   SF 6.63
  J1 MG996R  SF 14.00
  tipping SF 2.86
```
