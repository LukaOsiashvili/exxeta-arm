# ADR-0008: Structure of the PC software and the web app

- **Status:** accepted (implementer choice)
- **Spec:** §8, D10, D12

## 1. dora-rs version

Pinned to **dora-rs 1.1.0** (Python package) and **dora CLI 1.1.0**, the latest stable on 2026-11-15. It needs **Python 3.11 or newer**. The API used was checked against the repository docs: `Node()`, `for event in node`, `event["type"] == "INPUT"`, `node.send_output(id, pyarrow.Array, metadata)`, timers as `dora/timer/millis/N`.

## 2. Node logic is separated from dora

Every node is a small class with `on_input(name, data)` that calls `self.emit(output, data)`. It does not import dora. `exxeta_arm/runtime.py` connects such a class to a dora `Node`.

Why: the same classes run under `tools/run_local.py` (no dora needed) and in the tests, where the whole simulated dataflow runs on virtual time in about a second. The dora-specific code is 20 lines in one file, which is also what the tutorial explains.

`run_local.py` is a development aid only. It delivers messages depth-first in one thread, which is not how dora schedules separate processes.

## 3. Message format

- A list of numbers travels as an **Arrow float64 array** (`joint_goals`, `joint_targets`).
- Everything else travels as **one JSON string in an Arrow string array**.

JSON is slower than typed Arrow structs but readable in `dora` logs and trivial to extend, which matters more for a learning project at 50 Hz.

## 4. Where IK runs

The spec's node table gives IK to the `kinematics` node and trajectories to the `planner`. A Cartesian straight line needs hundreds of IK calls with continuity checks before the move may start, so the **planner calls the kinematics library directly**. The `kinematics` node still exists: it publishes the live end-effector pose and answers reachability queries from the web app's 3D target. Both use the same file, `exxeta_arm/kinematics.py`.

## 5. Who may change the arm's state

Only the `safety` node sends `control` commands to the bridge (connect, enable, disable, stop, clear). STOP is latched in the safety node **and** sent to the firmware. "Park & power off" is a planner move to the rest pose; when the planner reports it finished, the safety node disables the arm.

## 6. Web app: plain ES modules, no build step

- The app is static files in `webapp/` (HTML, CSS, JavaScript modules). There is nothing to build, so **Node.js is needed neither to run nor to modify it**. This goes one step further than the spec's "Node only to modify the UI".
- **three.js 0.169.0** and the **Roboto** font files are stored in `webapp/vendor/`, so the app works on a LAN without internet.
- A small built-in **URDF reader** (about 60 lines in `viewer.js`) replaces a URDF-loader library. This robot only needs revolute, prismatic, fixed and mimic joints, and one dependency less is one thing less to break.
- The server is **aiohttp** in its own thread, so a slow browser cannot stall the dora event loop.
- The font is set in exactly one place: `--font` in `webapp/css/style.css`.

## 7. Limits of the web app

- "Add to Home Screen" works (manifest and icons), but offline caching and install prompts need HTTPS, which a LAN address does not have. Documented, not solved.
- The access token is a convenience against accidental control, not real security: it travels unencrypted.
