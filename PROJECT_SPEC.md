# exxeta-arm: Project Specification

> **Audience:** the implementing agent (Fable) and the human team.
> **Status:** requirements v1.0, 2026-11-15. All decisions in [§2](#2-decision-log) were made by the project owner. Do not reverse them without asking.
> **Keywords:** **MUST** = required, **SHOULD** = strongly preferred (deviations must be justified in `docs/decisions/`), **MAY** = optional.

---

## 1. Overview

**exxeta-arm** is a low-cost, 3D-printed, 6-servo desktop robot arm (5 joints plus a gripper), inspired by the LeRobot **SO-100 / SO-101** arms. It is built entirely from parts that are easy to buy in **Georgia**.

The SO-100/101 relies on Feetech STS3215 smart servos, which aren't available locally. exxeta-arm uses **ordinary hobby PWM servos** instead (SG90, MG90S, S3003, MG995, MG996R, all 180° positional versions). These servos give **no position feedback**, so the design compensates in software and offers *optional* external joint sensing.

### 1.1 Goals

1. A **functional** arm that can reach points in its workspace, follow Cartesian moves, and pick up small light objects.
2. A **learning platform for the team**, mainly for **dora-rs**, the Rust-based robotics dataflow framework that serves as a lighter ROS 2 alternative. Kinematics, embedded firmware, CAD and electronics are secondary learning goals.
3. **Control from PC and mobile** through one responsive web app.
4. A **clean, solid-looking product**. All electronics, battery and wiring are hidden inside a heavy base, and nothing loose hangs off the arm.
5. **Fully parametric CAD**, so the team can change dimensions and regenerate every part.

### 1.2 Non-goals for v1 (possible future work, see [§13](#13-future-work-out-of-scope-for-v1))

- Leader arm or leader/follower teleoperation
- Hardware emergency-stop button (**dropped by owner decision**, see §2)
- Camera or computer-vision tasks
- LeRobot dataset recording or ML policy training
- Standalone operation without the PC (ESP32-only control)
- Current sensing, custom PCB

---

## 2. Decision Log

These are fixed inputs from the project owner.

| # | Topic | Decision |
|---|---|---|
| D1 | Kinematic layout | **5 joints + gripper = 6 servos**, SO-101-style: base yaw, shoulder pitch, elbow pitch, wrist pitch, wrist roll, gripper. |
| D2 | Servos | Only **SG90, MG90S, S3003, MG995, MG996R** (180° versions). The design assigns a model to each joint based on torque calculations. The team buys what the design specifies. |
| D3 | Joint feedback | **No servo modification ("surgery").** Primary mode is **open loop** with software safeguards. The design **MUST include optional mounts for external B10K rotary potentiometers** on the 5 arm joints. The arm must work fully without them. |
| D4 | Leader arm | **None.** Control comes only from the PC and mobile apps, with **FK and IK**. |
| D5 | Compute | **dora-rs runs on a PC.** The microcontroller is only a hardware bridge. **A PC is always required** for operation. |
| D6 | PC ↔ MCU link | **Both USB serial and Wi-Fi, switchable.** |
| D7 | Microcontroller | **ESP32 38-pin DevKit (classic ESP32-WROOM-32)** with Wi-Fi. The pin map must be documented so a different variant can be swapped in easily. |
| D8 | Firmware tooling | **PlatformIO**. The framework (Arduino-ESP32 or ESP-IDF) is the implementer's choice, with the choice justified. |
| D9 | Wi-Fi mode | Join a **saved network (station mode)**, and if none is found, fall back to the arm's **own hotspot (access point)** with a provisioning page. |
| D10 | dora node language | **Python only.** |
| D11 | Host OS | **Must run on Windows, macOS and Linux, including WSL2.** Team members use all of them. |
| D12 | Control UI | **One responsive web app** served from the PC, usable on desktop and phone browsers. |
| D13 | v1 extra features | **3D digital twin** (with a simulator mode) and **record & replay**. |
| D14 | CAD | **build123d is the source of truth.** A **Fusion 360** parametric model is also produced from the same parameters (via the Fusion 360 Python API, run with computer-use access the owner will grant). |
| D15 | 3D printing | **No printer owned yet**, so parts will be printed by a service or a friend. Design for a **generic 220 × 220 × 250 mm FDM** bed with tunable tolerances. |
| D16 | Size / payload | **Decided by the design.** Fable picks the largest reach that keeps a torque safety margin and documents the calculation. |
| D17 | Power | **Internal 2S Li-ion (18650) pack + DC jack**, all inside the base. The owner's 20,000 mAh USB power bank (5 V; one 2 A port and one 3 A port) **cannot power the servos** but MAY serve as an auxiliary charging source (§6.3). |
| D18 | Base controls | **Rocker switch (KCD1/KCD3 family)** for main power, **ESP32 voltage sensing** for battery level, and an **RGB status LED**. |
| D19 | E-stop | **No hardware E-stop.** Cutting servo power makes the arm fall, and keeping power on defeats the purpose. Safety is provided by the main rocker (hard kill), a software STOP, a watchdog and firmware limits (§10). |
| D20 | BOM sourcing | **Common generic parts only.** Each item needs search keywords and at least one alternative. No prices. |
| D21 | Team | **Mixed skill levels.** Docs must explain *why*, include step-by-step guides, and include learning exercises. |
| D22 | Version control | **None.** Do not run `git init` (workspace rule, see `D:\Robotics\CLAUDE.md`). |
| D23 | Gravity-compensation springs | **Not included by default.** If the torque budget fails, present the options to the team (§4.3) before adding a spring. |

---

## 3. System Architecture

```
┌──────────────────────── PC (Windows / macOS / Linux / WSL2) ────────────────────────┐
│                                   dora-rs dataflow                                  │
│                                                                                     │
│  ┌────────────┐   commands   ┌──────────┐  joint goals  ┌──────────┐               │
│  │ web_server │─────────────▶│ planner  │──────────────▶│  safety  │──┐            │
│  │ (HTTP+WS)  │◀─────┐       │ (traj.)  │◀──┐           │ (limits, │  │ safe joint │
│  └─────▲──────┘      │       └────▲─────┘   │           │  modes)  │  │ targets    │
│        │             │            │ IK/FK   │           └──────────┘  ▼            │
│        │             │       ┌────┴─────┐   │   ┌───────────┐   ┌─────────────┐    │
│        │             │       │kinematics│   │   │ recorder  │   │ arm_bridge  │    │
│        │             │       └──────────┘   │   │ (rec/play)│   │ OR          │    │
│        │             │                      │   └───────────┘   │ sim_bridge  │    │
│        │             └──── joint_state, status, battery ◀───────┤             │    │
│        │                                                         └──────┬──────┘    │
└────────┼────────────────────────────────────────────────────────────────┼───────────┘
         │ HTTP / WebSocket (LAN)                       USB serial  OR  Wi-Fi (UDP)
   ┌─────┴──────┐                                                         │
   │ Browser:   │                                       ┌─────────────────▼────────┐
   │ PC / phone │                                       │ ESP32 (in base)          │
   └────────────┘                                       │ servo PWM, interpolation,│
                                                        │ limits, watchdog, ADC    │
                                                        │ (pots, battery), RGB LED │
                                                        └──────────┬───────────────┘
                                                                   │ PWM 50 Hz
                                                       6 hobby servos (+ optional 5 pots)
```

Principles:
- **The ESP32 is a "smart I/O" device.** It runs no kinematics and no planning. It executes joint targets smoothly, enforces limits independently, and reports state.
- **Defence in depth.** Joint limits and speed limits are enforced in **both** the `safety` node and the firmware.
- **Hardware and simulation are interchangeable.** `sim_bridge` has exactly the same inputs and outputs as `arm_bridge`, so the whole stack runs without the arm.
- **Single source of geometry.** One parameter file drives the CAD, URDF, kinematics and web-app 3D model (§4.5).

---

## 4. Mechanical Design

### 4.1 Kinematic layout

| ID | Joint | Axis | Notes |
|---|---|---|---|
| J1 | `base_yaw` | vertical | Turntable on the base. Range is set by the cable loop, e.g. ±90° to ±120°. No slip ring. |
| J2 | `shoulder_pitch` | horizontal | Highest-loaded joint. |
| J3 | `elbow_pitch` | horizontal | |
| J4 | `wrist_pitch` | horizontal | |
| J5 | `wrist_roll` | along forearm/wrist axis | |
| J6 | `gripper` | (gripper actuation) | Open/close only. |

The geometry SHOULD resemble the SO-101's proportions and character, but all dimensions are derived from torque limits (§4.3). Do **not** copy SO-101 CAD files. Design original parts.

### 4.2 Servo reference data

Use these values as rated values, then **derate** them (§4.3). Cheap clones often deliver 50–70% of their rated torque.

| Model | Mass | Stall torque @4.8 V / @6 V | Speed @4.8 V / @6 V (s/60°) | Gears | Spline | Notes |
|---|---|---|---|---|---|---|
| SG90 | ~9 g | ~1.8 / ~2.2 kg·cm | 0.10 / 0.08 | plastic | 21T (micro) | Weak, gears strip |
| MG90S | ~13 g | ~1.8 / ~2.2 kg·cm | 0.10 / 0.08 | metal | 20T (micro) | Preferred micro servo |
| S3003 | ~37 g | ~3.2 / ~4.1 kg·cm | 0.23 / 0.19 | plastic | 25T (Futaba) | Standard size, slow |
| MG995 | ~55 g | ~8.5 / ~10 kg·cm | 0.20 / 0.16 | metal | 25T | Known for deadband and jitter |
| MG996R | ~55 g | ~9.4 / ~11 kg·cm | 0.17 / 0.14 | metal | 25T | Best available; stall current ~2.5 A |

The implementer MUST verify spline counts against real parts or datasheets. Parts SHOULD attach to the servo **through the servo's own horn** (round metal horns for the 25T servos), screwed to the printed part, rather than through a printed spline.

### 4.3 Torque budget and sizing (MUST be done first)

1. Write `tools/torque_budget.py`. It reads the parameter file and computes **static gravitational torque** at J2, J3 and J4 for the worst case (arm horizontal, fully extended, payload at the gripper). Masses come from CAD volumes × filament density × infill factor, plus servo and hardware masses.
2. **Safety factor:** required torque ≤ **50% of the rated stall torque at 6 V** for each joint, i.e. a safety factor of ≥ 2. J1 and J5 must also be checked against inertia and friction loads.
3. Choose the **servo per joint** and the **largest reach and payload** that satisfy the safety factor. The expected envelope is roughly **22–28 cm reach and 50–150 g payload**, but the calculation decides.
4. Output a table in `docs/design/torque_budget.md`: joint, servo, required torque, available derated torque, margin.
5. Expected (non-binding) outcome: J1 MG996R, J2 MG996R, J3 MG996R/MG995, J4 MG90S or S3003, J5 MG90S, J6 MG90S/SG90. Prefer **MG90S over SG90** and **MG996R over MG995**.
6. If J2 cannot meet the safety factor at a useful reach, **stop and present the options to the team**: shorter links, lighter structure, dual mirrored shoulder servos (risk: without feedback, mismatched servos fight each other and overheat), a counterbalance spring (D23), or a **parallelogram linkage** with the elbow servo moved down to the base, uArm/MeArm-style (much lower shoulder load, but the arm no longer looks like an SO-101). Do not choose on your own.

### 4.3.1 Joint construction pattern (one-sided servo output)

Hobby servos have one output shaft, so every pitch joint (J2–J4) MUST use the **fork (clevis) pattern**:
- The **proximal link** holds the servo body in a pocket (screwed through the mounting ears). The printed housing extends past the back of the servo and carries a **stub coaxial with the output shaft**.
- The **distal link** is a U-shaped fork. **Fork arm 1** is screwed to the servo horn. **Fork arm 2** carries a **bearing** that rides on the stub.
- The servo transmits torque only. All bending and radial loads go through the two supports.
- The optional pot (§4.5) sits on the stub/bearing side.

J1: the servo is vertical in the base, its horn drives the turntable, and the turntable bearing carries the arm's weight. J5: the servo lies axially in the forearm or wrist, its horn drives the gripper plate, and a ring or bearing takes the radial load. J6: the horn drives the gripper mechanism.

### 4.4 Structural requirements

- **No servo shaft carries bending loads.** Every pitch joint MUST have a **passive pivot bearing on the side opposite the servo horn** (e.g. 608, 623 or 625 bearings, or similar common sizes). The servo provides only torque.
- **Base yaw** MUST use a proper thrust and radial support, such as a lazy-susan bearing, a large thin-section bearing, or printed races with steel balls or a 608 centre. The J1 servo must not carry the arm's weight.
- **Fasteners:** standardize on **M3** (M2 or M2.5 only where micro servos require it). Use **captive hex nuts in printed pockets** by default, because the team may not have heat-set insert tools. Heat-set inserts MAY be offered as a parameter option.
- **Hard stops:** add mechanical end stops slightly beyond each joint's software limits where practical.
- **Rest pose:** design a folded **rest pose** in which the arm sits on a printed cradle or support on the base. This lets servo power be removed safely: the arm is parked before power-off and nothing falls.
- **Alignment marks:** each joint MUST have molded alignment notches at its zero position (and ideally at ±90°), for calibration without sensors (§7.4).
- **Cable management:** internal channels or clip paths for servo cables along every link. Leave enough slack at each joint for its full range, and avoid pinch points. J1 has a defined cable loop through the turntable centre.
- **Gripper:** type chosen by the design (parallel rack-and-pinion vs SO-101-style claw), with the rationale written down. Target an opening of ≥ 50 mm if feasible. Fingertips SHOULD be swappable and take a grip pad (printed TPU, rubber or tape).

### 4.5 Optional joint potentiometers (D3)

- Each of J1–J5 has a mount for a common **B10K rotary potentiometer** (6 mm D-shaft or knurled shaft. Document both, choose one as default), **coaxial with the joint axis** on the passive side.
- The pot **body** is fixed to one link. Its **shaft** is keyed into the other link through a printed socket with a slightly loose fit, so the pot **never carries structural load** and tolerates small misalignment.
- The pot mount MUST coexist with the passive bearing (e.g. a bearing around a printed hub and the pot behind it).
- When no pot is installed, a **printed cover plug** closes the mount cleanly.
- Pot wiring (3 wires per joint) follows the servo cable paths.

### 4.6 Base (electronics enclosure)

The base holds **everything** except the servos, pots and their cables:

- ESP32 38-pin DevKit, preferably on a **screw-terminal expansion board** (search: *"ESP32 38 pin expansion board"*) or soldered perfboard
- 2S2P 18650 battery pack with BMS, servo buck converter, logic buck converter, charger module, fuse
- **Main rocker switch** (KCD-family, panel cut-out), **DC input jack**, **USB-C panel-mount extension** to the ESP32's USB port (for serial control and flashing without opening the base)
- **RGB status LED** (WS2812/SK6812) behind a printed light-pipe or diffuser
- Servo and pot distribution board (3-pin headers)

Requirements:
- **Stability:** the base must not tip with the arm fully extended at maximum payload in any yaw direction. Compute the tipping moment and document it. Put the battery as low as possible. Offer optional desk clamp or screw holes and **rubber feet**.
- **Service access:** a removable bottom or rear panel for battery access, screwed rather than snapped. The ESP32 must be replaceable without desoldering.
- **Ventilation** for the buck converters, and thermal separation between converters and the battery.
- **Looks:** a clean, closed, consistent design language (matching fillets and chamfers), suitable for **two-colour printing** (body + accent). An optional "EXXETA" text emboss on the base, toggled by a parameter.
- **Footprint** is decided by the design. It SHOULD stay compact (target ≤ ~180 × 180 mm, but stability wins).

### 4.7 Printability (D15)

- Every part fits in **220 × 220 × 250 mm**.
- Design for **PLA+ or PETG**. Document the recommended material, wall count, infill and **print orientation** for every part in `docs/build/print_guide.md`.
- Minimize supports. Prefer splitting parts over heavy supports.
- **Tolerances are global parameters** (hole clearance, press-fit, nut-pocket clearance, bearing seat). Provide a small **tolerance test print** (bearing seat, nut pocket, servo pocket, pot socket, at several clearances) to print first at whichever service is used.
- Export one STL per part, named `<nn>_<part>_<qty>x.stl`, plus a STEP of the full assembly.

### 4.8 CAD deliverables (D14)

**build123d (source of truth)**
- `cad/params/arm_params.json` is the **single source** for link lengths, offsets, servo choices, bearing sizes, tolerances and feature toggles (pots on/off, emboss on/off, fastener type).
- One Python module per part, plus an assembly script that positions all parts at configurable joint angles.
- `tools/export_all.py` regenerates **every STL, the STEP assembly, the GLB/STL link meshes** for the URDF, and **PNG renders** for the docs.
- Automated checks: **interference check** across the joint range (sampled), **print-bed fit check**, and **mass estimate** fed into the torque budget.

**URDF / robot description (generated, never hand-edited)**
- `description/exxeta_arm.urdf` plus meshes, generated from the same parameters. Used by kinematics, the simulator and the web app.

**Fusion 360**
- `cad/fusion360/build_exxeta_arm.py` is a **Fusion 360 Python API script**. It reads `arm_params.json`, creates **User Parameters**, and builds a **native parametric** model with one component per part and Fusion joints for motion. It does not import STEP files as dumb bodies.
- The implementer runs it inside Fusion 360 through computer use (the owner grants access), saves the design, and exports a `.f3d` to `cad/fusion360/`.
- Acceptance: each part's bounding box and volume match build123d within a small tolerance (to be documented).

---

## 5. Electronics

### 5.1 ESP32 pin constraints (classic ESP32-WROOM-32, 38-pin DevKit)

- **Analog inputs MUST use ADC1** (GPIO 32, 33, 34, 35, 36/VP, 39/VN). **ADC2 cannot be used while Wi-Fi is active.** That gives exactly 6 channels: **5 joint pots + 1 battery voltage**. If more analog inputs are needed, or the ESP32 ADC proves too noisy or nonlinear, an **ADS1115** (I2C, 16-bit, 4 channels; common module) MAY be added. Document this as an option.
- **Servo PWM:** 6 output-capable GPIOs. Avoid **strapping pins** (0, 2, 5, 12, 15), **input-only pins** (34–39) and flash pins (6–11).
- **RGB LED** data on a free output GPIO.
- Produce `docs/hardware/pinmap.md`, which is the single source the firmware's config header follows.
- **3.3 V logic into 6 V servos:** usually works. Test it, and if any servo misbehaves, add a **74AHCT125** level shifter (a design option).

### 5.2 Servo signal and behaviour

- 50 Hz PWM. The default pulse range is ~500–2500 µs, but **every servo is calibrated individually** (min and max pulse, centre offset, direction).
- Firmware updates targets at **≥ 100 Hz internally** using velocity- and acceleration-limited interpolation, and writes pulse widths every PWM period.
- A **software deadband** stops tiny pulse changes, to reduce jitter and heating (MG995 especially).
- **Staged attach on boot:** start PWM on one servo at a time, with a delay between them, to limit inrush current and sudden jumps.

### 5.3 Wiring

- Servo rail ground and ESP32 ground are **common**.
- **Bulk capacitor(s)** (e.g. 1000–2200 µF, low ESR) at the servo distribution point.
- Wire gauges: battery and servo rail main lines ≈ 16–18 AWG, servo extension leads standard.
- Deliverables: a **wiring diagram** (SVG or KiCad schematic. A KiCad schematic is preferred if feasible) and a **perfboard or terminal-board layout** in `electronics/`.

---

## 6. Power System (D17, D18)

### 6.1 Architecture

```
 DC jack ──▶ 2S CC/CV charger ──┐
 (USB-C 5V ▶ 2S boost charger)┄┄┤ optional, power-bank input
                                ▼
                     2S2P 18650 pack + 2S BMS (balancing)
                                │
                             [fuse]
                                │
                      KCD rocker (main power)
                                │
              ┌─────────────────┼──────────────────┐
              ▼                 ▼                  ▼
   Servo buck 6.0 V      Logic buck 5 V      Divider ▶ ESP32 ADC1
   (≥10 A capable)       (≥1 A)  ▶ ESP32 5V  (battery voltage)
              │
   bulk caps ▶ servo distribution ▶ 6 servos
```

### 6.2 Requirements

- **Cells:** genuine **high-drain 18650** cells (≥ 10 A continuous each). Docs MUST warn about fake high-capacity cells (e.g. "9900 mAh" labels).
- **BMS:** 2S with **balancing**, continuous rating above the system's peak current.
- **Servo rail:** **6.0 V nominal** buck converter, sized for ≥ 10 A peaks with derating, because cheap modules are overrated. The implementer MUST check **dropout across the whole battery range** (2S ≈ 6.0–8.4 V). With non-synchronous bucks, 6.0 V output is impossible near empty. Choose a converter that can handle this, **or** set a lower servo voltage, **or** set a firmware low-battery cutoff that guarantees headroom. Document the choice.
- **Logic rail:** a separate small 5 V buck, so servo noise doesn't brown out the ESP32. **Prevent backfeeding** between the external 5 V and the PC's USB 5 V when the USB cable is connected (Schottky or ideal diode, verified against the DevKit's own circuit).
- **Fuse** on the pack output.
- **Main switch:** KCD-family rocker. A **KCD1 (~6 A) is too weak to carry the current directly.** Either use a **KCD3/KCD4 rated ≥ 15 A** in the battery line, or have the rocker switch a MOSFET load switch. Document the choice.
- **Charging:** via the **DC jack**, using a wall adapter suited to **Georgian mains (220–240 V, 50 Hz, type C/F plug)**. The implementer specifies the adapter voltage and current. Operating **while charging** MUST be safe and documented (simplest acceptable approach: the load always runs from the pack while the charger tops it up).
- **Power-bank input (SHOULD consider):** a USB-C 5 V → 2S boost charging module, so the owner's power bank or any phone charger can slowly top up the pack. If it adds too much complexity, document why it was rejected.
- **Battery monitoring:** the ESP32 reads the pack voltage through a divider into ADC1, with calibration and filtering, and converts it to a percentage using a 2S Li-ion curve. Warning threshold around 3.5 V/cell: the LED shows a warning and the app shows a banner. Critical threshold around 3.3 V/cell: the arm moves to the rest pose and refuses new motion. The BMS remains the hard cutoff.
- **Runtime estimate** in the docs (expected to be several hours of typical use).
- **Li-ion safety section** in the docs: no unattended charging, no damaged cells, use a cell holder or properly welded pack (never solder directly to cells), and correct polarity.

---

## 7. Firmware (ESP32, PlatformIO)

### 7.1 Responsibilities

1. Accept commands over **USB serial and Wi-Fi** using **one transport-agnostic protocol** (§7.2).
2. Drive 6 servos with interpolation, speed and acceleration limits, deadband and staged attach.
3. Enforce **joint limits and max speeds independently** of the PC.
4. Read optional pots and battery voltage, then filter and report them.
5. Run the **link watchdog**: if no valid command or heartbeat arrives for **500 ms** (configurable), **hold the last position** (do not detach, which would drop the arm), raise `LINK_LOST`, and refuse new motion until the controller re-enables the arm.
6. Drive the RGB status LED with documented colour codes: booting, waiting for PC, connected-idle, enabled/moving, warning (battery), fault, Wi-Fi provisioning AP.
7. Store configuration in **NVS**: Wi-Fi credentials (several networks), per-servo calibration, joint limits, pot calibration, device name, auth token.
8. Handle **Wi-Fi provisioning**: try the saved networks in station mode, and if none connect within N seconds, start the AP **`exxeta-arm-XXXX`** with a captive setup page (Wi-Fi only, no control). Credentials MAY also be set over the serial CLI. Advertise **mDNS** as `exxeta-arm.local` (`exxeta-arm-XXXX.local` if several arms are present).
9. Provide a **human-readable debug CLI** over USB serial (help, status, move single joint, calibration helpers, Wi-Fi setup). It must not interfere with the binary protocol, e.g. a mode switch or a distinct framing prefix.
10. **OTA firmware update** over Wi-Fi: SHOULD.

### 7.2 Protocol

- **Versioned and documented** in `docs/software/protocol.md`, with identical semantics over serial and Wi-Fi.
- The implementer chooses the encoding. Recommended: compact binary frames (**COBS** framing, **CRC-16**, sequence number, message type).
- Wi-Fi transport: **UDP**, latest-wins for streamed targets and state, with acknowledgement and retry for configuration and commands. A short shared-token handshake prevents accidental control from another PC on the LAN.
- **Single controller:** only one PC session can control the arm at a time. A second one gets `BUSY` until the first releases control or its link times out.
- Rates: commands ~50 Hz, state ~50 Hz (configurable).
- Message types, at minimum: `HELLO/HANDSHAKE`, `ENABLE`, `DISABLE`, `SET_JOINT_TARGETS` (6 targets + optional per-joint max speed), `STOP` (controlled stop: decelerate and hold), `HEARTBEAT`, `GET/SET_CONFIG`, `CALIBRATION` commands, `STATE` (commanded positions, interpolated positions, pot readings with validity flags, battery voltage and percent, fault flags, uptime, link stats), `LOG/ERROR`.
- **Shared test vectors:** a JSON file of encoded frames used by both the Python tests and the firmware's native unit tests.

### 7.3 Boot and startup without feedback

There is no absolute position at power-on unless pots are installed. The startup sequence MUST be:

1. **On boot:** no PWM output. The LED shows "waiting for PC".
2. **The PC connects** and the user presses **Enable** in the app.
3. **Without pots:** the firmware assumes the arm is in the **rest pose** (the user is instructed to always park before power-off), attaches the servos one by one at the rest-pose angles, then ramps slowly.
   **With pots:** it reads the actual joint angles, attaches each servo at its measured angle, and moves nowhere.
4. **Before power-off:** the app provides a **"Park & power off"** action that moves to the rest pose and then disables.

### 7.4 Calibration (no sensors needed)

There is a guided calibration flow in the web app, with a CLI fallback, for each joint:
1. Jog the joint until its alignment notches line up at 0°, then save the offset.
2. Optionally confirm at ±90° notches to fit the pulse-to-angle scale.
3. Set and verify direction and safe min/max limits.
4. With pots installed, record pot readings at the same reference points to fit the ADC-to-angle mapping.

The result is stored in the firmware's NVS **and** exported to a calibration file on the PC (`config/calibration/<device>.json`).

### 7.5 Testing

- The protocol encoder and decoder plus the interpolation logic must be unit-testable on the host (PlatformIO `native` environment).
- `docs/firmware/bench_test.md` covers a step-by-step bench bring-up: ESP32 + 1 servo → all servos → full arm.

---

## 8. PC Software (dora-rs, Python only)

### 8.1 General

- Use the **latest stable dora-rs** release at implementation time, and **pin** both the CLI and the Python package versions. **Verify every API against the official docs (dora-rs.ai and the GitHub repo).** The dora API has changed between releases, so don't rely on memory.
- Python environment management: **`uv`** (preferred) or venv + pip. Use one `pyproject.toml`.
- **Cross-platform (D11):** all nodes must run natively on **Windows, macOS and Linux**. Avoid dependencies without wheels for all three (e.g. prefer pure-NumPy kinematics over heavy native libraries).
- **WSL2 caveats MUST be documented**, and the system should work around them:
  - USB serial needs `usbipd-win`, so recommend the **Wi-Fi transport** under WSL.
  - WSL2 NAT blocks inbound LAN traffic. Phones can't reach the web server, and ESP32 UDP or mDNS may not reach WSL. Document the `netsh interface portproxy` and firewall steps, or *mirrored networking* where available (Windows 11 22H2+; the owner's PC is Windows 10, so document the portproxy route).
- Units: **SI internally** (radians, metres, seconds). Degrees and mm only in the UI.
- All tunable values live in YAML config under `config/`, not hard-coded.

### 8.2 Nodes

| Node | Purpose | Key inputs → outputs |
|---|---|---|
| `arm_bridge` | Talks to the ESP32 over serial **or** Wi-Fi (config + runtime switch). Handles the handshake, reconnection, heartbeat and calibration upload. | `joint_targets`, `control` → `joint_state`, `device_status`, `battery` |
| `sim_bridge` | Drop-in replacement for `arm_bridge`. Simulates servos with a speed/acceleration model and deadband. MAY optionally use **MuJoCo** (MJCF generated from the URDF) for gravity and torque checks. | same as `arm_bridge` |
| `state_estimator` | Merges commanded and interpolated positions with pot readings (when present). Every joint is flagged `measured` or `estimated`. | `joint_state` → `estimated_state` |
| `kinematics` | **FK** and **IK** from generated parameters. **Analytic IK** for this 5-DoF geometry (base yaw from atan2, planar 2-link + wrist pitch, wrist roll), with a numeric fallback. Targets are position + approach (pitch) angle + roll. Unreachable targets are reported clearly rather than clamped silently. | `cartesian_goal`, `estimated_state` → `joint_goal`, `ee_pose` |
| `planner` | Smooth trajectories: joint-space moves (trapezoidal or minimum-jerk velocity profiles), **Cartesian straight-line moves** (sampled IK with continuity checks), **jogging** (continuous velocity commands from the UI), and named poses (home, ready, rest). | goals/jog → `joint_goals` stream |
| `safety` | The last gate before the bridge. Enforces joint, speed and acceleration limits, workspace keep-outs (table plane, base cylinder, simple self-collision checks), the mode state machine (DISABLED, IDLE, MOVING, STOPPED, FAULT), STOP handling and low-battery behaviour. | `joint_goals` → `joint_targets`, `safety_status` |
| `recorder` | **Record & replay** (§8.4). | UI commands, `estimated_state` → goals to `planner` |
| `web_server` | Serves the web app and a **WebSocket** bridge between browsers and dora. Handles multi-client access with a single **control lock**. Must not block the dora event loop. | everything UI-related |

- Provide dataflows `dataflows/hardware.yml` and `dataflows/sim.yml`, which differ only in the bridge node.
- MAY: an optional **Rerun** visualisation node for debugging (dora has Rerun integration) as a learning extra.

### 8.3 Web app (D12)

- Served by `web_server` at `http://<pc-ip>:<port>`. On startup, the terminal and the app print a **QR code** so phones can open it.
- **Responsive**: works with a mouse and keyboard on desktop and with touch on a phone. "Add to Home Screen" support (manifest + icons). Full PWA features need HTTPS, which a LAN IP doesn't have, so document this limitation.
- **Visual design:** **brutalist**. Black and white only, with thick borders, hard edges, no rounded corners, gradients or soft shadows, and a bold grid layout. Colour appears only for essential status (STOP, faults, low battery). Font: **Roboto** for now, defined in one place so it's easy to change.
- **Running the system must not require Node.js.** The built frontend is stored in the project. Node is only needed to *modify* the UI. The framework is the implementer's choice (keep it simple; vanilla TS + Vite or Svelte is suggested). Use **three.js** with a URDF loader for 3D.
- **Features:**
  - Connection panel: choose the **transport (USB serial port / Wi-Fi host)**, connect and disconnect, and show link quality, latency, firmware version and device name
  - **STOP button always visible** (controlled stop) on every screen. Keyboard: **Space/Esc** on desktop.
  - Enable, Disable, and **Park & power off**
  - **Joint control:** sliders per joint with limits shown, plus numeric entry
  - **Cartesian control:** jog buttons for X/Y/Z, pitch, roll and gripper. **On-screen joysticks** on mobile. On desktop, **drag an IK target gizmo** in the 3D view and see whether it is reachable.
  - **3D digital twin:** live pose of the arm. When pots are present, a **ghost overlay** shows commanded vs measured. Orbit, zoom, and preset views.
  - Named poses: save, go to, rename, delete
  - **Record & replay UI** (§8.4)
  - Status: battery %, mode, faults, a log panel, and a simulation/hardware indicator
  - **Calibration wizard** (§7.4)
  - Settings: speed scaling, limits (read-only unless in an "advanced" mode), Wi-Fi provisioning helper
  - Multi-client: any number of viewers, but **one controller** ("Take control" button, released automatically on disconnect)
- Optional simple **access token** for the web app, set in config.

### 8.4 Record & replay (D13)

- **Waypoint programs:** a list of poses (joint or Cartesian) with per-segment speed, gripper state and dwell time. Created by jogging and pressing "Add waypoint". Editable in the UI (reorder, edit, delete).
- **Continuous recordings:** timestamped joint states captured while the user jogs. When pots are installed, a **"limp teach" mode** MAY be offered: servos are released (only in a safe pose, with a clear warning), the user moves the arm by hand, and the pots record the motion.
- Playback: play, pause, stop, **speed scaling**, loop, and step-through. **Every playback passes through `planner` and `safety`.**
- Storage: human-readable **JSON** (waypoint programs) and **CSV** (continuous recordings) under `data/recordings/`, with a version field. Design the format so a later LeRobot-dataset exporter is possible.

---

## 9. Simulation and Digital Twin

- `sim.yml` runs the **entire stack, including the web app, with no hardware**. This is the default way for team members without an arm to develop.
- The simulated servos use the same speed model and limits as the real ones, so behaviour matches.
- **MuJoCo is optional** (it has pip wheels for all three OSes). If it is implemented, use it to validate the torque budget dynamically and show joint torques in the UI.
- The 3D twin in the web app uses the **generated URDF and meshes**, so it updates automatically when the CAD parameters change.

---

## 10. Safety (without a hardware E-stop, D19)

1. **Main rocker** = hard power kill. Document that the arm goes limp and drops: keep hands and objects clear, and park first when possible.
2. **Software STOP** (UI button + keyboard) → controlled deceleration and hold. It latches until it's acknowledged.
3. **Firmware watchdog** → hold position plus `LINK_LOST` latch (§7.1).
4. **Limits in two layers**: the `safety` node and the firmware.
5. **Low-battery behaviour** (§6.2).
6. **Startup procedure** (§7.3) prevents sudden jumps.
7. **Speed caps:** the default speed for new users is conservative, and higher speeds are enabled in settings.
8. **Docs safety page:** pinch points, Li-ion handling, servo overheating (hobby servos burn out when stalled against an obstacle, so don't leave them holding loads for long), and running-while-charging notes.

---

## 11. Project Structure (proposed)

```
exxeta-arm/
├── README.md                  # overview + quick start (per OS)
├── PROJECT_SPEC.md            # this file
├── CLAUDE.md                  # agent conventions for this project
├── cad/
│   ├── params/arm_params.json # SINGLE SOURCE of geometry and options
│   ├── parts/                 # build123d part modules
│   ├── assembly.py
│   ├── fusion360/             # Fusion API script + exported .f3d
│   └── export/                # generated STL / STEP / renders
├── description/               # generated URDF + meshes (+ MJCF)
├── electronics/               # wiring diagram, schematic, perfboard layout
├── firmware/                  # PlatformIO project
├── dora/
│   ├── nodes/<node_name>/     # one package per node
│   ├── dataflows/hardware.yml, sim.yml
│   └── pyproject.toml
├── webapp/                    # frontend source + built dist/
├── config/                    # YAML configs, calibration files
├── data/recordings/
├── tools/                     # torque_budget.py, export_all.py, gen_urdf.py, …
├── tests/                     # pytest (kinematics, planner, safety, protocol, sim e2e)
└── docs/
    ├── decisions/             # ADRs for every implementer choice
    ├── design/                # torque budget, stability, kinematics derivation
    ├── build/                 # BOM, print guide, assembly guide, wiring guide
    ├── hardware/              # pin map
    ├── firmware/
    ├── software/              # architecture, protocol, dora tutorial
    └── learning/              # exercises for the team
```

---

## 12. Deliverables and Acceptance Criteria

### 12.1 Documentation (D21: mixed-skill team)

- **README quick start** for Windows, macOS, Linux and WSL2: install → run `sim.yml` → open the web app on PC and phone.
- **BOM** (`docs/build/bom.md`): every item with quantity, spec, **search keywords**, an **alternative**, and the joint or subsystem it belongs to. This includes the servos (with spares), bearings, screws, nuts, electronics modules, cells, cables, connectors and adapter. No prices.
- **Print guide, assembly guide** (step-by-step with renders), **wiring guide**, **calibration guide**, **troubleshooting** (jitter, brownouts, Wi-Fi, WSL networking, MG995 deadband).
- **Kinematics explainer:** frame assignment (DH or URDF-style), FK derivation, analytic IK derivation with diagrams.
- **dora-rs tutorial using this project:** nodes, dataflows, Arrow messages, how to add a node.
- **Learning exercises** (with hints and reference solutions kept separate). For example: write a node that draws a square with the end effector, add gamepad control, add a new safety rule, add a simulated obstacle, make a pick-and-place waypoint program, and a camera-node starter.
- **ADRs** in `docs/decisions/` for every choice this spec leaves to the implementer.

### 12.2 Acceptance criteria

1. `tools/export_all.py` regenerates all STL, STEP, URDF and render outputs from `arm_params.json` without errors. The interference and bed-fit checks pass.
2. The torque budget shows a safety factor of ≥ 2 on every joint for the chosen reach and payload.
3. `dora` + `sim.yml` runs on Windows, macOS and Linux. The web app works from a phone on the same LAN.
4. FK/IK round-trip tests pass across the sampled workspace. Unreachable targets are reported.
5. The firmware builds in PlatformIO. Native unit tests pass. The shared protocol test vectors pass in both Python and C++.
6. On hardware: switching between serial and Wi-Fi works without reflashing. Watchdog, STOP, limits and low-battery behaviour are verified with a documented test checklist.
7. Record & replay works in both sim and hardware.
8. The Fusion 360 model is generated by script from the same parameters and matches the build123d model.
9. With pots disabled in the params, the arm builds and runs open loop. With pots enabled, the twin shows measured angles.

### 12.3 Suggested implementation phases

| Phase | Content | Exit criterion |
|---|---|---|
| 0 | Params file, torque budget, servo selection, ADRs | Team approves the sizing |
| 1 | build123d CAD v1, tolerance test print, single-joint test rig | Test prints fit |
| 2 | Firmware bring-up + protocol over serial, 1 servo bench test | Servo controlled from a Python script |
| 3 | dora skeleton: `sim_bridge`, `safety`, `web_server`, joint sliders | Sim controllable from phone |
| 4 | Kinematics, planner, Cartesian jog, 3D twin | IK drag works in sim |
| 5 | Full electronics + base, assembly, calibration | Real arm moves safely |
| 6 | Wi-Fi transport + provisioning + mDNS | Wireless control works |
| 7 | Record & replay, optional pots support | Programs replay on hardware |
| 8 | Fusion 360 model, docs polish, exercises | Acceptance criteria met |

---

## 13. Future Work (out of scope for v1)

- Hardware E-stop (if a small, suitable button becomes available; D19 may be revisited)
- Current sensing (INA219/INA226) for stall and collision detection
- Camera + vision (object detection, ArUco-based joint estimation)
- LeRobot dataset export and imitation-learning policies
- Leader arm (potentiometer-based)
- Custom PCB replacing perfboard and modules
- ESP32 standalone mini-UI
- ROS 2 bridge

---

## 14. Instructions for the Implementing Agent

0. **Priority: deliver a functional, physically buildable v1.** Getting the core working (arm built, moving safely, controlled from the app via dora) beats completeness. Any SHOULD or MAY item that threatens this may be deferred. List deferred items in `docs/decisions/deferred.md`. The team will refine and extend later.
1. Read this whole spec and `D:\Robotics\CLAUDE.md` first. **No git.**
2. Start with **Phase 0** and present the torque budget and sizing to the team **before** generating the full CAD.
3. When something is ambiguous or a MUST conflicts with physics or part availability, **stop and ask**. Don't silently deviate. Record every resolved choice as an ADR.
4. Verify external facts (dora-rs API, library versions, servo splines, module pinouts) against current sources rather than memory.
5. Prefer simple, well-commented code that a mixed-skill team can read. Every node and firmware module gets a short header explaining its role in the system.
6. Keep `arm_params.json` the only place geometry is defined. Never hand-edit generated files.
