# ADR-0002: Bearings, gripper and pot mounts

- **Status:** accepted by the owner, 2026-11-15, with changes (608ZZ preferred; no J5 pot in v1)
- **Spec:** §4.3.1, §4.4, §4.5

## 1. Passive pivot bearings: 608ZZ on J2, J3 and J4

**608ZZ (8 × 22 × 7 mm, 12 g), one size on all three pitch joints.** It is the skateboard bearing: sold in every bearing, skate and hardware shop, usually in packs of eight.

Phase 0 proposed 6802ZZ (15 mm bore) because the pot shaft could then pass *through* a hollow printed stub inside the bearing. The owner asked for 608ZZ, since 6802 may be hard to find locally. The joint was redesigned so the 608 works:

- The pivot is a short **printed pin with an M3 screw through its centre**. The pin goes through the bearing from the outside and the screw clamps it to the servo housing, so the steel screw carries the load, not an 8 mm plastic stub.
- The optional pot no longer passes through the bearing. It sits in a small pod outside the fork arm and is turned by the head of the pivot pin through a loose printed coupler. The pot still carries no load and tolerates misalignment (§4.5).
- Cost: 4.4 g more per joint than a 6802. With the 50 g rating there is room for it (shoulder SF 2.33).

Fallback sizes (bore and seat are parameters; listed in the BOM): **6800ZZ** (10 × 19 × 5), **6000ZZ** (10 × 26 × 8), **688ZZ** (8 × 16 × 5).

## 2. Gripper type

The SO-101-style claw proposed here was replaced by a rack-and-pinion parallel gripper at the owner's request. See [ADR-0006](ADR-0006-rack-and-pinion-gripper.md).

## 3. Pots: J2–J4 coaxial, J1 gear-coupled, J5 none (deviation from §4.5 and D3)

§4.5 asks for a coaxial pot mount on each of J1–J5. On J1 and J5 the servo body itself sits on the joint axis, so there is no passive side to put a pot on.

- **J2, J3, J4:** coaxial pot on the bearing side, as specified.
- **J1:** pot beside the axis, driven by a printed 1:1 gear pair (accepted by the owner). Gear backlash adds roughly ±1° of measurement error.
- **J5: no pot in v1** (owner decision). The wrist is where mass hurts most, and roll does not change the gravity load, so feedback there is worth the least. Its ADC1 pin stays free. Options for that pin later: a J5 pot (a mount would have to be designed), or monitoring the battery mid-point to see cell balance.
