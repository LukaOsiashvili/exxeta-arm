# Not verified on this machine

Things that could not be checked during implementation and should be tested first by the team. Updated at the end of every phase.

## Phase 0 (sizing)

| # | What | Why it could not be verified | How to verify |
|---|---|---|---|
| 1 | Real stall torque of the purchased MG996R and MG90S | No hardware. Clones often deliver 50–70 % of the datasheet value. | Bench test before assembly: hang a known weight on a 10 cm lever and find where the servo stops lifting at 6.0 V. The shoulder needs about 4.7 kgf·cm; a healthy MG996R should lift at least 8. |
| 2 | Printed-part masses | No CAD yet; hand estimates with about 15 % contingency. | Phase 1 CAD replaces them automatically; weigh the first prints on kitchen scales. |
| 3 | **Dropout of the servo-rail converter on a 2S pack** | Cheap modules do not publish it. The firmware thresholds assume at most 1.2 V. | Bench supply (or a nearly empty pack) at **6.6 V** input, output set to 6.0 V, **3 A** load (for example two 4.7 ohm 10 W resistors in parallel): the output must stay >= 5.4 V. Repeat at 7.0 V (expect >= 5.8 V). If it fails, use the UBEC alternative or raise `critical_v`. |
| 4 | Servo stall currents (2.5 A / 0.9 A assumed) | Datasheets and sellers disagree (MG90S: 0.6–0.9 A; MG996R: 1.4–2.5 A). | Measure with a multimeter in series while holding the horn. |
| 5 | Spline tooth counts (MG90S 20T vs 21T) | Sources disagree. | Count on the real part. Not critical: parts attach through the servo's own horn. |
| 6 | Base mass and tipping margin | Base not designed yet. | Recomputed from CAD in Phase 1/5. |
| 7 | Charger float behaviour (pack held at 8.2 V while the adapter stays plugged in) | No hardware. | Set the module to 8.20 V / 1.5 A with no pack connected, then charge and confirm with a multimeter that the pack settles at 8.2 V and both cells agree within 0.05 V. |
| 8 | 15 A fuse rides through switch-on inrush | No hardware. | Switch on 20 times with all servos connected; the fuse must not blow. |

## Phase 1 (CAD)

| # | What | Why it could not be verified | How to verify |
|---|---|---|---|
| 9 | **Servo and horn dimensions** in `servo_mech` (ear height, horn face height, horn shape, shaft offset) | Taken from common drawings; clones differ by up to 1 mm and I could not find a dimensioned manufacturer drawing online. | Measure your servos with calipers, correct `arm_params.json`, then print the **tolerance test** and the **single-joint test rig** before any large part. |
| 10 | Fit of bearing seat, nut pockets, horn pockets, pot socket | No printer. | Tolerance test plate (part 18): pick the column that fits and set `tolerances`. |
| 11 | Smoothness and play of the printed J1 ball race | No print. | Assemble base, turntable, balls and clamp ring; it must turn freely with no rocking. |
| 12 | Rack-and-pinion gripper: mesh, friction, backlash | No print. | Fingers must slide by hand before the servo is fitted; if tight, raise the 0.15 mm backlash in `rack_sketch`/`gear_sketch`. |
| 13 | Printability of the 41 mm bridge over the MG996R pockets and of the gripper body | No printer. | Check the first print; add supports in the slicer if the bridge sags. |
| 14 | Cable lengths: the MG90S leads (about 25 cm) will not reach the base from the wrist | No hardware. | Buy servo extension leads (see BOM); route and check slack over the whole joint range. |
| 15 | One tiny face of one part does not triangulate in the STL export (warning only) | The mesher reports it but the part is not identified yet. | Slicers repair this automatically; check the slicer shows no holes. |

## Firmware, PC software, web app

| # | What | Why it could not be verified | How to verify |
|---|---|---|---|
| 16 | **Anything on hardware**: servo motion, staged attach, watchdog, STOP, limits, battery thresholds, Wi-Fi, hotspot page, mDNS, OTA, LED | No ESP32 or servos connected. The firmware only builds (66 % flash). | `docs/firmware/bench_test.md`, step by step, then the 14-point checklist at its end. |
| 17 | Firmware native unit tests (`pio test -e native`) | No host C++ compiler on the build machine. | Run on any PC with gcc or clang. They use the same vectors the Python tests pass. |
| 18 | **Running under dora-rs** (`dora run dataflows/sim.yml`) | dora is not installed here (by instruction). The nodes were run with `tools/run_local.py` and in the tests; the dora glue in `runtime.py` (about 20 lines) was written from the 1.x API docs and never executed. | Install dora CLI and package 1.1.0, run the sim dataflow, open the app. Check first: node paths in the YAML (`../nodes/...`, relative to the YAML file) and that each node finds its Python environment. |
| 19 | `arm_bridge` against a real ESP32 (handshake, latency, reconnect, calibration upload) | No hardware. Protocol encode/decode is tested; the transport code is not. | `python tools/bench_client.py <port>` first (simpler), then the hardware dataflow. |
| 20 | macOS, Linux, WSL2 | Only Windows 10 with Python 3.12 was available. | Run the tests and the simulation on each. Nothing OS-specific is used on purpose. |
| 21 | Web app on a real phone, with touch | Checked in a desktop browser at phone size only. | Open the printed address on a phone on the same Wi-Fi; try the joysticks and STOP. |
| 22 | Web app features exercised only lightly | Enable, move to a pose, the IK target and the layout were checked in the browser. Record/replay, park, STOP/clear and Cartesian moves were checked through the automated tests, not by clicking. The calibration panel, connection panel, joysticks and pose rename/delete were not exercised. | Click through every tab in simulation. |
| 23 | Fusion 360: joints and the second script | The main script ran and its volumes match build123d (ADR-0009). Not tried: dragging the joints, joint limits (none are set), and `build_native_skeleton.py`. | Drag each joint in Fusion; run the skeleton script in an empty Hybrid Design if it is wanted. |
| 24 | Wiring diagram and wiring guide | Drawn from the design, never built. | Build on the bench in the order of the guide, measuring each rail. |
| 25 | Default servo pulses for "joint at 0" (J2 1400, J3 1111, J6 611 us) and the assembly instructions built on them | Derived from the joint ranges; no servo to try. | Check on the test rig that the pulse gives a horn position from which the whole joint range is reachable. |
