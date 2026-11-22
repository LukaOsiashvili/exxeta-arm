# Bill of materials

> No prices (D20). Electronics modules are re-checked when the base interior is designed; nothing here has been bought or tested yet.

## Servos

| Qty | Item | Spec | Used for | Search keywords | Alternative |
|---|---|---|---|---|---|
| 3 (+1 spare) | MG996R | 180° positional (not "360° continuous"), metal gear, 25T | J1, J2, J3 | `MG996R 180 degree servo metal gear` | MG995 180° (weaker, more jitter) |
| 3 (+2 spare) | MG90S | 180° positional, metal gear | J4, J5, J6 | `MG90S 180 metal gear micro servo` | SG90 (plastic gears, strips: last resort) |
| 3 (+1 spare) | Metal round servo horn, 25T | aluminium disc, about 20 mm, M3 threaded holes | J1, J2, J3 | `25T aluminium servo horn round disc MG996R` | the plastic round horn from the servo bag (weaker) |

Buy servos from a seller with good reviews. Cheap clones often deliver only 50–70 % of rated torque (see the torque budget).

## Bearings

| Qty | Item | Spec | Used for | Search keywords | Alternative |
|---|---|---|---|---|---|
| 3 (+1 spare) | 608ZZ ball bearing | 8 × 22 × 7 mm, metal shields | passive pivot of J2, J3, J4 | `608ZZ bearing`, `skateboard bearing 608` | 608-2RS (rubber seals, slightly more drag) |

**Why 608ZZ:** it is the most common bearing there is (skateboards, scooters, fidget spinners), so any bearing, skate or hardware shop has it.

**If 608 is unavailable**, these also work after changing the bearing size in `arm_params.json` (`hardware.bearing_pitch`) and re-exporting the parts:

| Fallback | Size (mm) | Note |
|---|---|---|
| 6800ZZ | 10 × 19 × 5 | lighter; pivot pin becomes 10 mm |
| 6000ZZ | 10 × 26 × 8 | very common in motors and tools; slightly larger fork arm |
| 688ZZ | 8 × 16 × 5 | same 8 mm pin, smaller and lighter; common in 3D-printer parts |

## Battery

| Qty | Item | Spec | Used for | Search keywords | Alternative |
|---|---|---|---|---|---|
| 4 | 18650 Li-ion cell, **high-drain** | **3000 mAh preferred** (about 20 % more runtime); 2500 mAh is the baseline. ≥ 15 A continuous. Flat-top, unprotected. All four the same model and batch. | 2S2P pack | `Samsung 30Q`, `LG HG2`, `Sony VTC6` (3000 mAh); `Samsung 25R`, `Sony VTC5` (2500 mAh) | any genuine high-drain cell ≥ 2000 mAh, ≥ 15 A |
| 2 | 18650 holder, 2 cells | spring or leaf contacts, thick leads; rewire for 2S2P | pack | `18650 battery holder 2 slot` | 1 × 4-slot holder |

### Fake-cell warning

- **Any 18650 labelled above about 3600 mAh is fake.** "9900 mAh", "6800 mAh" and similar labels (often "UltraFire" or no brand) are recycled or tiny cells in a new wrapper. They cannot deliver servo currents and are a fire risk.
- No real 18650 combines very high capacity with very high current. A real high-drain cell is 2000–3000 mAh.
- A genuine cell weighs **44–48 g**. Fakes are often under 40 g: weigh them on kitchen scales.
- Buy from a vape shop or a known battery seller, not from a random marketplace listing. Reject cells with dents, rust, torn wrap or different codes printed on cells from the "same" batch.
- Never mix cells of different model, age or charge state in one pack.

## Fasteners

Everything is M3 except the screws that come with the micro servos. An **M3 assortment box** (screws 6–20 mm, nuts) covers it.

| Qty | Item | Used for | Search keywords | Alternative |
|---|---|---|---|---|
| 14 | M3 × 8 screw, pan or cap head | forks to metal horns (8), turntable to J1 horn (4), spare | `M3x8 screw` | M3 × 10 with a washer |
| 16 | M3 × 10 | MG996R ears (12), J1 bracket to base (4) | `M3x10 screw` | |
| 16 | M3 × 12 | clamp ring (6), pivot covers (6), bottom panel (4) | `M3x12 screw` | |
| 3 | M3 × 16 | through the pivot pins | `M3x16 screw` | |
| 40 | M3 hex nut | captive nuts everywhere | `M3 nut` | |
| 2 | M2 × 10 self-tapping, pan head | wrist cap through the J5 servo ears | `M2x10 self tapping screw` | the longest screws from the servo bags |
| 50 | Steel ball, 6 mm | J1 turntable bearing (46 used) | `6mm steel ball bearing balls` | 1/4 inch (6.35 mm) bicycle balls: set `turntable.ball_d` to 6.35 and re-export |
| 20 | Small zip tie, 2.5 mm | cable routing | `cable tie 2.5 x 100` | |
| 4 | Rubber foot, 12 mm, stick-on | under the base | `self adhesive rubber feet 12mm` | |

## Electronics

Module choices are re-checked when the base interior is designed. Buy the converters first and run the dropout test in `docs/decisions/unverified.md`.

| Qty | Item | Spec | Used for | Search keywords | Alternative |
|---|---|---|---|---|---|
| 1 | ESP32 DevKit, 38 pin | ESP32-WROOM-32, USB | controller | `ESP32 DevKit 38 pin WROOM-32` | 30-pin DevKit (re-check the pin map) |
| 1 | Screw-terminal expansion board | for the 38-pin DevKit | wiring without soldering to the ESP32 | `ESP32 38 pin expansion board screw terminal` | perfboard with pin sockets |
| 1 | Servo buck converter | input from ≤ 6 V, output 6.0 V, ≥ 8 A continuous | servo rail | `DC-DC 12A buck 4.5-30V adjustable` | RC UBEC ≥ 10 A, 2S–6S, 6 V output |
| 1 | Logic buck converter | ≥ 1 A, adjustable | ESP32 supply | `MP1584 buck module`, `Mini360` | any small 5 V buck |
| 1 | CC/CV charger module | adjustable voltage and current, ≥ 2 A | charging | `XL4015 CC CV 5A module` | an "8.4 V 2S Li-ion charger" adapter into the jack (then no module; it charges to 4.2 V per cell) |
| 1 | 2S BMS with balancing | 20 A | cell protection | `2S BMS 20A balance 8.4V` | 2S 10 A (trips before the fuse in a stall) |
| 1 | Blade fuse holder + 15 A fuse | inline, 16 AWG leads | pack output | `inline blade fuse holder 16AWG`, `15A blade fuse` | |
| 1 | Rocker switch KCD3 | ≥ 15 A | main power | `KCD3 rocker switch 15A` | KCD4 (larger, 16–30 A). **Not KCD1** |
| 1 | DC panel jack | 5.5 × 2.1 mm | charging input | `DC panel jack 5.5 2.1` | |
| 1 | DC adapter | 12 V, ≥ 2 A, 5.5 × 2.1 mm, EU plug, 220–240 V | charging | `12V 2A power adapter 5.5x2.1` | 9–24 V adapters also work with the CC/CV module |
| 1 | USB-C panel-mount extension | short, to the DevKit's USB port type | flashing and serial without opening the base | `USB C panel mount extension` (micro-USB or USB-C plug to match the DevKit) | |
| 1 | Electrolytic capacitor | 1000–2200 µF, ≥ 10 V, low ESR | servo rail | `2200uF 16V low ESR` | two 1000 µF in parallel |
| 1 | Schottky diode | 1N5819 | logic rail | `1N5819` | SS14 |
| 1 each | Resistor 100 kΩ, 33 kΩ, 330 Ω; capacitor 100 nF | | divider, LED | resistor kit | |
| 1 | RGB LED | WS2812B or SK6812, single, on a small board | status | `WS2812B single LED module` | |
| 3–4 | Servo extension lead, 30 cm | 3-pin | J3–J6 | `servo extension cable 30cm` | |
| 2 m each | Silicone wire 16 AWG red, black | | main power | `16AWG silicone wire` | 14 AWG |
| 4 (optional) | Rotary pot B10K, 6 mm D-shaft, 15 mm shaft | WH148 type, linear | joint feedback J1–J4 | `WH148 B10K potentiometer 15mm` | knurled shaft (drill the coupler to 6 mm) |
| 1 (optional) | USB-C 2S boost charger | 5 V in, 8.4 V out | charging from a power bank, arm parked | `Type-C 2S 8.4V boost charger IP2326` | |

## Printing

About 520 g of PETG for the standard arm, plus about 140 g for the tolerance test and the test rig, plus a little TPU for the two fingertips. See the [print guide](print_guide.md).
