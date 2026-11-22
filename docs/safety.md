# Safety

This arm has **no hardware emergency stop** (owner decision D19). Read this page before powering it.

## The four ways to stop

| What | How | What happens |
|---|---|---|
| **STOP** (normal) | red bar in the app, or **Space / Esc** on a keyboard. Works from every connected device, with or without control. | The arm brakes smoothly and holds. It stays stopped until someone presses ACKNOWLEDGE. |
| **Link lost** (automatic) | PC software crashes, cable pulled, Wi-Fi drops | After 0.5 s the firmware holds position by itself and blinks red. Acknowledge to continue. |
| **Battery empty** (automatic) | pack below 6.6 V | The arm parks itself in the rest pose and refuses to move. |
| **Main switch** (last resort) | rocker on the base | Power is cut. **The arm goes limp and falls.** Keep hands and objects out from under it. |

Whenever possible use **PARK & POWER OFF** before switching off: the arm folds onto its stops first, then the servos release and nothing falls.

## Start-up rule

The servos have no position feedback. At ENABLE the firmware **assumes the arm is in the rest pose**. If it is not, the joints snap to where the firmware thinks they are. So: always park before switching off, and if the arm was moved by hand, fold it back into the rest pose before pressing ENABLE.

## Hands and pinch points

- The forks at shoulder, elbow and wrist close like scissors. Keep fingers out while the arm is enabled.
- The gripper is weak (about 6 N per finger) but the gear and racks can pinch.
- Keep the area the arm can reach (27 cm around the base, table level upwards) free of cups, cables and faces.
- New users: leave the speed at its default (40 %).

## Servos get hot

A hobby servo that is pushed against something it cannot move draws its full stall current and **burns out in well under a minute**. So:

- If a joint hums loudly and does not move, press STOP and find out why.
- Do not leave the arm holding a load at full stretch for long. Park it when idle.
- The gripper keeps squeezing after it has closed on an object. Do not hold objects for many minutes.
- The software limits keep the hand off the table and the base, but it knows nothing about objects on the table.

## Battery (lithium-ion)

- Genuine high-drain 18650 cells only, in holders. See the fake-cell warning in the [BOM](build/bom.md).
- Never charge unattended. Unplug the adapter when the app says the charger is connected and the battery is full.
- Charging while the arm runs is allowed: the arm always runs from the pack and the charger tops it up.
- A cell that is dented, rusty, hot or swollen goes to battery recycling, not back into the arm.
- Do not bypass the fuse or the BMS.
- If something smells or smokes: main switch off, unplug the charger, move the base onto a non-flammable surface if that is safe to do.

## Limits of the software

- The keep-out zones are simple shapes (table plane, a cylinder around the base, the shoulder). They do not model the forearm against the base in every pose, or anything you put on the table.
- Limits are enforced twice (PC and firmware), but both believe the calibration. A badly calibrated joint has wrong limits.
- Nothing here has been tested on hardware yet: run the [hardware test checklist](firmware/bench_test.md) before trusting any of it.
