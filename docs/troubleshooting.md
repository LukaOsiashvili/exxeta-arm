# Troubleshooting

## Servos

| Symptom | Likely cause | Fix |
|---|---|---|
| Servo twitches or ignores commands | no common ground | join servo-rail ground and ESP32 ground |
| All servos jitter when one moves | supply sags | fit the 1000–2200 µF capacitor at the servo headers; check the buck is the ≥ 8 A type; thicker wires |
| ESP32 restarts when the arm moves (brownout) | logic rail shares the servo supply, or weak USB power | ESP32 must be fed from its own 5 V buck through the diode |
| A servo hums at rest | it is holding a load, or is pushed against a stop | normal under load; if against a stop, fix the calibration or the limits |
| MG995 buzzes or hunts around its position | wide dead band and a noisy feedback pot, typical for MG995 | use MG996R; a tighter joint (no play) helps; the firmware ignores pulse changes below 2 µs |
| Joint moves the wrong way | servo direction | recalibrate: a negative µs-per-degree is stored and handled |
| Servo does nothing on 3.3 V signal | rare input stage | add a 74AHCT125 level shifter (5 V side on the servo rail) |
| Arm jumps at ENABLE | it was not in the rest pose | park before switching off; fold it into the rest pose by hand |
| Arm sags or stalls at full reach | weak clone servo, low battery, too much payload | measure the servo on the test rig; charge; stay at or below 50 g |
| Gripper does not close fully or binds | rack friction, pinion one tooth off | clean and grease the guides; refit the pinion with the fingers closed |

## Power

| Symptom | Cause | Fix |
|---|---|---|
| App says "servo power is off" | main switch off, fuse blown, BMS tripped | check in that order. A tripped BMS resets when the charger is plugged in briefly |
| Fuse blows at switch-on | short, or reversed capacitor | check with a multimeter before replacing the fuse |
| Battery percentage jumps while moving | normal voltage sag under load | the firmware filters it over 3 s; only the slow value counts |
| Arm parks itself early | converter dropout too high: the rail collapses before 6.6 V | run the dropout test (`docs/decisions/unverified.md` item 3); use the UBEC alternative |
| Charger LED never goes out | the arm is drawing current | normal, see the power budget section 6 |

## Connection

| Symptom | Cause | Fix |
|---|---|---|
| No serial port in the app | driver missing (CP2102 or CH340), charge-only USB cable | install the driver, use a data cable |
| "Arm is controlled by another PC (BUSY)" | another session holds control | stop the other software, or wait 3 s after it has gone |
| "Wrong auth token" | token in `config/arm.yaml` differs from the arm's | set the same number, or `token 0` in the firmware CLI |
| Wi-Fi: "no answer from the arm" | wrong address, different network, firewall | use the IP shown by the CLI `status`; allow UDP port 47800 |
| `exxeta-arm.local` not found | mDNS not available (WSL2, some Windows and Android setups) | use the IP address |
| Arm starts its own hotspot | saved network not found | join `exxeta-arm-XXXX`, open `192.168.4.1`, enter the network |
| LINK_LOST now and then over Wi-Fi | weak signal | move closer, or use USB; the app shows link quality and latency |

## Web app

| Symptom | Cause | Fix |
|---|---|---|
| Phone cannot open the address | PC firewall, or guest Wi-Fi that isolates devices | allow Python on private networks (TCP 8080); use the same normal Wi-Fi |
| Buttons do nothing | you do not hold control | press TAKE CONTROL (one device at a time) |
| "Arm is not enabled" in the log | commands sent while disabled | press ENABLE |
| A move is refused with "hand would hit the table / base" | safety keep-out zone | choose another pose; this is working as intended |
| "Line not possible at N %" | a point on the straight line is unreachable | move in joint mode, or choose a closer target |
| 3D view empty | `description/` missing | run `python tools/export_all.py` |

## WSL2

See [WSL2 notes](software/wsl2.md). Short version: use the Wi-Fi transport, connect to the arm by IP, and forward port 8080 from Windows if phones must reach the app.

## dora

| Symptom | Fix |
|---|---|
| `dora: command not found` | install the dora CLI 1.1.0 and reopen the terminal |
| A node fails with `ModuleNotFoundError` | run inside the project's Python environment (`pip install -e dora`) or use `dora run … --uv` |
| Port 8080 already in use | another copy is running, or change `web.port` in `config/arm.yaml` |
| To see what a node prints | `dora logs <dataflow> <node id>`; `python tools/run_local.py` shows everything in one terminal |
