# Calibration guide

Every hobby servo is a little different: "1500 µs" is not exactly the middle, and the degrees per microsecond vary. Calibration tells the firmware, for each joint, **which pulse means 0°** and **how many microseconds make one degree**. Two points per joint are enough. No sensors are needed: you line up the alignment marks by eye.

The result is stored in the arm (flash) and copied to `config/calibration/<device name>.json` on the PC.

## Where the marks are

| Joint | 0° means | Marks |
|---|---|---|
| J1 base | arm facing the front of the base | three ticks on the turntable rim (0°, ±90°), notch on the clamp ring |
| J2 shoulder | upper arm straight up | three ticks on the fork's bearing arm, line on the tower |
| J3 elbow | forearm in line with the upper arm | ticks on the forearm's fork, line on the elbow housing |
| J4 wrist | hand in line with the forearm | ticks on the wrist fork, line on the wrist block |
| J5 roll | fingers move left-right when the arm is upright | three grooves on the gripper skirt, line on the wrist cap |
| J6 gripper | fingers closed (pads touching) | none: use closed and fully open |

## Procedure (web app, SETUP tab, section CALIBRATION)

The arm must be **connected, disabled and you must hold control**. Support the arm with one hand: during calibration only one servo is powered.

For each joint:

1. Choose the joint. The pulse starts at 1500 µs, the servo's centre, which is the middle of the joint's range (not 0° for J2, J3 and J6).
2. Press the −50 … +50 buttons until the 0° marks line up exactly. Press **1 · SET 0°**.
3. Jog to the +90° mark (J2: lean forward; J3, J4: bend forward). Leave the reference at 90 and press **2 · SET REF**.
   - J1: use the +90° tick (arm pointing left).
   - J6: jog until the fingers are fully open; enter **160** as the reference.
   - If a joint cannot reach +90°, use −90° or any angle you can measure, and enter that number.
4. Press **3 · SAVE TO ARM**. The app shows the result, for example "0° = 1482 µs, 10.6 µs per degree". A negative number means the servo turns the other way; that is fine and handled automatically.

Then press DISABLE (releases the servo) and go on to the next joint.

## Check

1. Fold the arm into the rest pose, press ENABLE. No joint may jump by more than a few degrees.
2. POSES → ZERO (the arm points straight up, slowly). All 0° marks should line up within about 2°.
3. POSES → HOME, then drive J2 to 90° with the slider: the upper arm should be horizontal.

If a joint is off, repeat its calibration.

## Without the app

Firmware CLI: `pulse <joint> <µs>` drives one servo with a raw pulse. Note the two values, compute µs per degree = (pulse at reference − pulse at 0°) ÷ reference angle, and write them into `config/calibration/<name>.json`; the bridge uploads the file at the next connect.

## Pots (optional)

With pots fitted, each pot needs the same two points (ADC reading at 0° and readings per degree). The firmware and the protocol support this (`POT_CAL`), and the file format has a `pots` section, but **the app has no pot wizard yet**: see `docs/decisions/deferred.md`.
