# What is where, and why

```
exxeta-arm/
├── README.md              start here
├── PROJECT_SPEC.md        the requirements (written by the owner)
├── CLAUDE.md              short rules for AI agents working on the project
├── cad/                   the mechanical design
├── description/           the robot model for software (generated)
├── electronics/           wiring diagram
├── firmware/              program for the ESP32
├── dora/                  program for the PC
├── webapp/                the control app shown in the browser
├── config/                settings you may change
├── data/                  recordings made with the app
├── tools/                 scripts that calculate, generate and check
├── tests/                 automatic tests
├── docs/                  all documentation
└── .venv/                 Python environment of this project (not part of the design; can be deleted and recreated)
```

## cad/ : the mechanical design

| Path | What | Edit by hand? |
|---|---|---|
| `cad/params/arm_params.json` | **The single source of truth**: link lengths, servo per joint, servo dimensions, tolerances, options. Everything else is derived from it. | **yes** |
| `cad/common.py` | Shared CAD helpers and the reusable building blocks (servo housing, fork). | yes |
| `cad/parts/*.py` | One file per printed part. The comment at the top says what the part does. | yes |
| `cad/assembly.py` | Puts the parts together at given joint angles; defines the link frames. | yes |
| `cad/requirements.txt` | Python packages needed for the CAD tools. | yes |
| `cad/fusion360/` | Script that builds the Fusion 360 model, and its output. | yes |
| `cad/export/stl/` | **The print-ready STL files**, one per part, with a `README.md` listing them. | no (generated) |
| `cad/export/exxeta_arm.step` | The whole arm as one STEP file, for any CAD program. | no |
| `cad/export/fusion_links/` | One exact STEP file per link, read by the Fusion script. | no |
| `cad/export/masses.json` | Mass and centre of mass of every part, used by the torque budget. | no |

Why a parameter file plus code instead of a drawn CAD model: change one number, run one command, and every part, the software's robot model and the strength calculation follow.

## description/ : the robot as software sees it (generated)

`exxeta_arm.urdf` describes links and joints; `meshes/*.stl` are the shapes of whole links **including servos**; `robot_info.json` holds a few extra facts (gripper travel per servo angle). Used by the web app's 3D view. **These STL files are not for printing.**

## electronics/

`wiring_diagram.svg`. There is no custom circuit board: the design uses ready-made modules.

## firmware/ : the ESP32 program (PlatformIO)

| Path | What |
|---|---|
| `platformio.ini` | Build settings, pinned versions. |
| `src/main.cpp` | Everything that touches hardware: servos, sensors, LED, serial, Wi-Fi, setup hotspot, debug commands. |
| `lib/armcore/` | Pure logic with no hardware in it (message format, motion smoothing), so it can be tested on a PC. |
| `include/pinmap.h` | Which pin does what. |
| `include/arm_defaults.h` | Limits and thresholds, generated from `arm_params.json`. |
| `test/` | Unit tests for `lib/armcore`. |
| `.pio/` | Build output (generated, can be deleted). |

## dora/ : the PC program

| Path | What |
|---|---|
| `dataflows/sim.yml`, `hardware.yml` | The "wiring diagram" of the software: which node talks to which. The two files differ in one line. |
| `nodes/<name>/main.py` | One small program per job: `arm_bridge` or `sim_bridge` (the arm, real or simulated), `state_estimator`, `kinematics`, `planner`, `safety`, `recorder`, `web_server`. |
| `exxeta_arm/` | The shared library the nodes use: `protocol.py` (messages to the ESP32), `kinematics.py`, `planner.py`, `safety.py`, `sim.py` (simulated arm), `servo_model.py`, `device.py` (serial and Wi-Fi), `robot.py` (reads the parameter file), `runtime.py` (connection to dora). |
| `pyproject.toml` | Python packages needed, with pinned dora version. |

Why nodes and a library: the logic can be tested without dora or hardware; the nodes stay a few lines long.

## webapp/ : the browser app

Plain files, no build step: `index.html`, `css/style.css` (look; the font is set in one place), `js/app.js` (behaviour), `js/viewer.js` (3D view), `vendor/` (three.js and the Roboto font, stored locally so the app works without internet), `manifest.json` and `icons/` (for "Add to Home Screen").

## config/ and data/

- `config/arm.yaml`: runtime settings (web port, USB or Wi-Fi, default speed, simulation options).
- `config/poses.json`: poses you saved in the app (created on first save).
- `config/calibration/<arm name>.json`: servo calibration, written by the app (created on first calibration).
- `data/recordings/`: programs (`.json`) and continuous recordings (`.csv`) made in the app.

## tools/ : scripts

| Script | What it does |
|---|---|
| `export_all.py` | **The main one.** Rebuilds all STL/STEP/URDF/renders and the parts lists, and checks that every part fits the printer, nothing collides and every joint is strong enough. |
| `torque_budget.py`, `power_budget.py` | The strength and battery calculations; they write `docs/design/*_budget.md`. |
| `gen_urdf.py` | Writes `description/` (called by `export_all.py`). |
| `gen_firmware_defaults.py` | Writes the firmware's defaults and the shared protocol test data. |
| `render.py` | Makes the pictures in `docs/img/`. |
| `run_local.py` | Runs the PC program without dora (for development). |
| `bench_client.py` | Smallest possible program to move one servo from the PC. |
| `arm_params.py` | Shared loader for the parameter file. |

## tests/

`test_core.py` (protocol, kinematics, planner, safety, simulator), `test_e2e_sim.py` (the whole simulated system), `protocol_vectors.json` (example messages shared with the firmware tests; generated).

## docs/

| Path | What |
|---|---|
| `build/` | **To build the arm:** `bom.md` (what to buy), `print_guide.md`, `wiring_guide.md`, `assembly_guide.md`, `calibration_guide.md`. |
| `safety.md`, `troubleshooting.md` | Read the first before powering anything. |
| `design/` | Why the arm is the size it is: `torque_budget.md`, `power_budget.md`, `cad_checks.md`, `kinematics.md`. |
| `decisions/` | One short record per design choice (`ADR-…`), plus `deferred.md` (not done yet) and `unverified.md` (not tested yet, and how to test). |
| `hardware/pinmap.md`, `firmware/bench_test.md` | Pins; first power-up step by step and the hardware test checklist. |
| `software/` | `dora_tutorial.md`, `protocol.md`, `wsl2.md`. |
| `learning/` | Exercises and solutions. |
| `img/` | Renders (generated). |
