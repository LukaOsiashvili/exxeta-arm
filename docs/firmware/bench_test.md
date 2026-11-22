# Firmware bench bring-up and hardware test checklist

Do the steps in order. Each one adds a single new thing, so a problem is easy to locate. **Nothing below has been run on hardware yet.**

## Status LED

| LED | Meaning |
|---|---|
| white | booting |
| blue, slow blink | waiting for the PC |
| dim cyan | PC connected, servos off |
| green | enabled, idle |
| bright green | moving |
| yellow, slow blink | battery low |
| red, fast blink | STOP latched, link lost or battery empty |
| magenta, slow blink | Wi-Fi setup hotspot is active |

## Debug CLI

Open a serial monitor at 115200 baud (`pio device monitor`) and type `help`. Commands: `status`, `enable`, `disable`, `stop`, `clear`, `move <joint 1-6> <deg>`, `pulse <joint 1-6> <µs>`, `wifi <ssid> <password>`, `name <name>`, `token <number>`, `save`, `reset`, `reboot`.

## Step 1: ESP32 alone

1. Connect the DevKit by USB. Nothing else is wired.
2. `cd firmware`, `pio run -t upload`, `pio device monitor`.
3. Expect the banner `# exxeta-arm firmware 0.1.0`. With no saved network the arm starts the hotspot `exxeta-arm-XXXX` after about 10 s per saved network (immediately if none).
4. Type `status`. Battery reads about 0 V and faults show `0x0008` (servo power off). That is correct: there is no battery.

## Step 2: one servo on the bench supply

1. Power **one MG90S** from 5–6 V. Join the supply ground to an ESP32 GND pin. Signal wire to GPIO 13.
2. Because there is no battery divider yet, the firmware refuses `enable`. Use the raw calibration command instead: `pulse 1 1500`. The servo moves to its centre.
3. Try `pulse 1 1000` and `pulse 1 2000`. `disable` releases the servo.
4. Fit the **battery divider** (100 kΩ from the supply positive to GPIO 32, 33 kΩ and 100 nF from GPIO 32 to GND). With the supply at 7–8.4 V `status` must show the voltage within 0.2 V. Never connect more than 12 V to the divider.

## Step 3: control from Python

With the divider reading above 6.6 V:

```bash
python tools/bench_client.py COM5
```

(Use your port name.) The script connects, enables, moves joint 1 by ±20° and prints the state stream. Then pull the USB data connection or stop the script with Ctrl+C **without** disabling: within half a second the LED must blink red (LINK_LOST) and the servo must stay where it is.

## Step 4: single-joint test rig

Mount an MG996R in the printed test rig (parts 19, 20). Repeat step 3 with joint 2. Then hang a known weight on the arm at the 10 cm hole and find the largest weight the servo lifts at 6.0 V: weight in kg × 10 = torque in kgf·cm. A healthy MG996R should lift 0.8 kg or more; the design needs 0.49 kg.

## Step 5: all six servos, then the arm

1. Wire all six servos to the distribution board. Power from the real battery through fuse, switch and the 6.0 V converter.
2. `enable`: listen for six servos starting one after the other (staged attach, 150 ms apart).
3. Assemble the arm **in the rest pose** and calibrate (see the calibration guide).

## Hardware test checklist (acceptance criterion 6)

| # | Test | Expected | OK |
|---|---|---|---|
| 1 | Switch between USB serial and Wi-Fi in the app, no reflash | both connect; state stream continues | ☐ |
| 2 | Second PC connects while the first controls | second gets "BUSY" | ☐ |
| 3 | Kill the PC software while moving | arm decelerates and holds within 0.5 s; LED red; no drop | ☐ |
| 4 | Restart the software, press ACKNOWLEDGE | arm accepts commands again, no jump | ☐ |
| 5 | STOP button (and Space key) during a fast move | smooth stop; further commands refused until acknowledged | ☐ |
| 6 | Command a joint beyond its limit from the CLI (`move 2 150`) | stops at the limit; LIMIT_CLAMPED shown | ☐ |
| 7 | Main switch off while enabled, USB still connected | app shows "servo power is off"; enable is refused | ☐ |
| 8 | Battery at 7.0 V (bench supply in place of the pack) | yellow LED, banner in the app | ☐ |
| 9 | Battery at 6.5 V for more than 5 s | arm parks itself and refuses motion | ☐ |
| 10 | Enable from the rest pose | no visible jump on any joint | ☐ |
| 11 | Park & power off | arm moves to rest, then servos release; arm stays on its stops | ☐ |
| 12 | Wi-Fi setup: no saved network | hotspot appears; page saves credentials; arm joins after restart | ☐ |
| 13 | `exxeta-arm.local` resolves on the LAN | connect by name works (not under WSL2) | ☐ |
| 14 | Charger plugged in at full charge | app shows "charger connected" | ☐ |
