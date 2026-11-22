# Power budget

> **Generated file. Do not edit by hand.** Regenerate with `python tools/power_budget.py`.
> Source: `cad/params/arm_params.json` (design revision `0.3-phase1`).

## 1. Pack

- 2S2P 18650, 2500 mAh high-drain cells (>= 20 A continuous each): **5.0 Ah, 36 Wh**, about 27 Wh usable down to the critical threshold.
- Voltage: 8.4 V full, 7.2 V nominal, **7.0 V warning**, **6.6 V critical** (park and refuse motion), 5.5 V BMS hard cut-off.

## 2. Load scenarios

Servo rail 6 V (converter efficiency 88 %). Logic: ESP32 with Wi-Fi + LED = 1.1 W. Battery current is given at the warning voltage (7.0 V), where it is highest.

| Scenario | Servo rail current (A) | Battery power (W) | Battery current (A) | Runtime (h) | Basis |
|---|---|---|---|---|---|
| Parked / disabled (no PWM, servos idle) | 0.00 | 1.4 | 0.20 | 19.7 | servos are not driven; only the ESP32 runs |
| Enabled, arm upright, nothing held | 0.06 | 1.8 | 0.25 | 15.2 | almost no gravity torque; servos just hold |
| Typical work (average of moving, holding, pauses) | 1.44 | 11.2 | 1.60 | 2.4 | assumed: joints average half of their worst-case torque |
| Holding rated payload at full reach (worst static pose) | 2.83 | 20.6 | 2.95 | 1.3 | torques from the torque budget |
| All six servos stalled at once (fault case) | 10.26 | 71.3 | 10.19 | 0.4 | sizes the fuse, BMS, switch and converter peak |

**Expected runtime: about 2 h of typical use**, about 1.3 h if the arm held the worst-case pose the whole time.

## 3. Component ratings

| Component | Rating | Must carry | OK? | Comment |
|---|---|---|---|---|
| Cells (per pack) | 40 A | 10.2 A | yes | all-stall fault current, shared by the parallel cells |
| BMS continuous | 20 A | 10.2 A | yes | must not trip before the fuse in a stall |
| Main rocker switch | 15 A | 15.0 A | yes | must survive anything the fuse lets through |
| Servo converter continuous | 8 A | 2.8 A | yes | worst static pose, continuous |
| Servo converter peak | 12 A | 10.3 A | yes | all-stall, seconds only; bulk capacitors help with shorter spikes |
| Fuse | 15 A | 10.2 A all-stall, 2.9 A normal | yes | clear of the all-stall peak and of capacitor inrush, so it only blows on a real short; below the BMS rating |
| Main wiring (16 AWG) | 22 A | 15.0 A | yes | the fuse must protect the wire: wire rating above fuse rating |

## 4. Servo-rail dropout (2S pack vs 6.0 V rail)

A buck converter can only step *down*, and it loses some voltage itself (its "dropout"). Near empty the pack voltage comes close to 6.0 V, so the rail can sag. Cheap modules rarely publish their dropout, so the table shows a good module (0.5 V) and a poor one (1.2 V). **The real value must be measured on the bench.**

| Pack state | Pack voltage under load (V) | Rail, good module (V) | Rail, poor module (V) | MG996R torque, poor module |
|---|---|---|---|---|
| full | 8.4 | 6.0 | 6.0 | 100 % |
| nominal | 7.2 | 6.0 | 6.0 | 100 % |
| warning | 7.0 | 6.0 | 5.8 | 98 % |
| critical: park | 6.6 | 6.0 | 5.4 | 93 % |

Decision (ADR-0003): keep the 6.0 V set-point and accept a sag to about 5.4 V near empty. All servos are rated 4.8-6 V, so the sag is harmless; it costs a few percent of torque exactly when the app is already telling the user to recharge.

### Firmware thresholds (consistent with the sag)

| Threshold | Pack voltage | Per cell | Worst-case rail | Action |
|---|---|---|---|---|
| Warning | 7 V | 3.50 V | 5.8 V | LED warning, banner in the app |
| Critical | 6.6 V | 3.30 V | 5.4 V | move to rest pose, then refuse motion |
| BMS cut-off | 5.5 V | 2.75 V | off | hard cut by the BMS (arm goes limp: this is why we park first) |

- The critical threshold is chosen so that even a poor converter still gives the servos 5.4 V, comfortably above their 4.8 V minimum, with torque to finish parking.
- The voltage is low-pass filtered (time constant 3 s) with 0.2 V hysteresis, because servo current spikes pull the pack down briefly and must not trigger a park.
- Below 4.5 V the firmware concludes that the main switch is off (ESP32 running from USB) and refuses to enable.

## 5. Battery voltage sensing

Divider 100 kΩ / 33 kΩ with 100 nF to ground, taken **after** the main switch (no drain when off).

- At 8.4 V the ADC pin sees 2.08 V; at 6.6 V it sees 1.64 V. Both lie in the linear part of the ESP32 ADC (about 0.15-2.45 V at 11 dB attenuation).
- Divider current: 63 µA.
- Reading near 0 V while USB is connected means the main switch is off: the firmware then refuses to enable the servos.

## 6. Charging, and charging while running

- Adapter: 12 V DC, >= 2 A, 5.5 x 2.1 mm, EU plug (type C/F). Internal CC/CV module set to **8.2 V / 1.5 A** (4.10 V per cell).
- Charge time from the critical threshold: about 3.2 h (the 1.3 factor covers the slow constant-voltage tail).
- Charge current per cell: 0.75 A (gentle; typical cells allow 1-2 A or more).

### What happens when the arm runs while the charger is connected

The load always runs from the pack; the charger feeds the same node. Checked behaviour:

1. **The CC/CV module has no end-of-charge detection at all.** It is a voltage-limited, current-limited supply. With or without a load it simply holds its set voltage for as long as the adapter is plugged in. So the concern is real: the pack *will* be held at the set voltage indefinitely. A running arm does not make this worse; it only means the module's 'charging' LED (which just shows current flowing) never goes out.
2. **It cannot overcharge.** The pack can never exceed the set voltage (8.2 V), and the BMS independently cuts off any single cell above about 4.25 V.
3. **Why 8.2 V and not 8.4 V.** What ages Li-ion cells is sitting for long periods at 4.2 V. Holding 4.10 V per cell is the usual stand-by (float) level for packs that stay on a supply; it costs roughly 10 % of capacity and turns 'left it plugged in over the weekend' from slow damage into a non-event.
4. **Net current while working.** Typical use draws 1.6 A and the charger supplies 1.5 A, so during work the pack drains slowly (-0.1 A net) and fills whenever the arm pauses.
5. **Firmware help.** When the filtered pack voltage is above 8.1 V the firmware reports 'charger connected / full', and the app shows a reminder to unplug.
6. **Rule for the team:** unplug the adapter when the app says full, and never charge unattended (safety page).

### Why not a power-path circuit

A power path would feed the arm from the adapter and charge the pack separately. Rejected for v1: (a) the servos can pull 10 A peaks at 6 V, far more than a 12 V / 2 A adapter delivers, so the pack has to stay in the circuit anyway; (b) the usual diode-OR costs 0.4-0.5 V in the battery line, which is exactly the headroom the 6.0 V servo rail lacks near empty (section 4); (c) more parts to wire wrongly.

### Two side effects to know

- **Balancing.** Typical 2S balance boards only bleed a cell once it reaches 4.20 V, so at 4.10 V per cell they stay inactive. Use four cells of the same type, age and batch, and check both cell voltages with a multimeter every few months (they should agree within 0.05 V). The ADC pin freed by dropping the J5 pot can optionally monitor the pack mid-point.
- **Optional USB-C charger (IP2326 type).** That module *does* terminate on low current, and a running arm confuses it. If it is fitted: charge from USB-C only with the arm parked and disabled.
