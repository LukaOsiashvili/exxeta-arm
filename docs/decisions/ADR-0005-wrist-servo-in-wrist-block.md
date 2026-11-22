# ADR-0005: The wrist-pitch servo sits in the wrist block, not in the forearm

- **Status:** accepted (minor deviation from the wording of §4.3.1, same intent)
- **Spec:** §4.3.1 says the *proximal* link holds the servo and the *distal* link is the fork.

## Decision

At J2 and J3 the spec's pattern is used as written. At **J4 it is mirrored**: the forearm (proximal) ends in the fork, and the wrist block (distal) holds the J4 servo. Everything else about the pattern is unchanged: one fork arm is driven by the servo horn, the other carries a 608 bearing on a pivot pin, and the servo shaft sees torque only.

## Why

- The wrist block has to hold the J5 (roll) servo anyway. Putting the J4 servo in the same block lets the two MG90S sit back to back, which makes the hand about 12 mm shorter. Every millimetre there is lever arm for the payload on the shoulder and the wrist.
- One compact block instead of a servo housing on the forearm plus a second fork on the wrist: fewer and lighter parts where mass matters most.
- Mechanically it is the same joint. Which side the servo body is on makes no difference to the loads.

## Consequence

The J4 servo cable moves with the hand instead of with the forearm. It runs through the hollow forearm together with the J5 and J6 cables, so nothing changes for cable management.

## Related details at J4, J5 and J6 (micro servos)

- The MG90S comes with plastic horns only. The printed part has a **pocket in the shape of the horn**; the horn transmits torque through the pocket and its own screw holds it on the shaft.
- J5: the gripper turns on a printed **journal** (part of the wrist cap that also clamps the servo), so side loads bypass the servo shaft.
- J5 and J6 use the double-arm horn **cut to 12 mm** with side cutters, because the full horn does not fit inside the gripper body and the pinion.
