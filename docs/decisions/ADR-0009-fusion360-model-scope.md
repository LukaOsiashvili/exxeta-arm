# ADR-0009: How the Fusion 360 model is built

- **Status:** accepted by the owner's request for "as detailed a model as possible" (2026-11-15). It deviates from the wording of D14 / §4.8; see below.
- **Spec:** D14, §4.8 ("native parametric model … does not import STEP files as dumb bodies … bounding box and volume match build123d")

## Decision

`cad/fusion360/build_exxeta_arm.py` builds a **full-detail assembly from exact geometry**:

- one component per link: base, turntable, upper_arm, forearm, wrist, gripper, pinion, finger_a, finger_b;
- each contains the exact part geometry (every pocket, hole and gear tooth), read from the per-link STEP files that `tools/export_all.py` writes to `cad/export/fusion_links/`. Printed parts and bought parts (servo and bearing stand-ins) are separate sub-components;
- joints: J1–J5 revolute, the pinion, and a slider for each finger, so the arm can be posed by dragging;
- the main dimensions as User Parameters, **for reference only**.

Run on 2026-11-15 in Fusion (Hybrid Design): 9 components, 8 joints, saved as `exxeta-arm` in the owner's Default Project and exported to `cad/fusion360/exxeta_arm.f3d`.

## Check against build123d

From `fusion_report.json` (links without bought parts compare directly):

| Link | Fusion volume | build123d volume | Fusion bounding box | build123d |
|---|---|---|---|---|
| forearm | 28.82 cm³ | 28.81 cm³ | 26.22 × 65.22 × 106.22 mm | 26.2 × 65.2 × 106.2 mm |
| pinion | 0.86 cm³ | 0.83 cm³ | 20.0 × 19.8 × 4.7 mm | same part |
| finger + fingertip | 5.45 cm³ | 5.47 cm³ | | |

Agreement is within about 0.5 % on the larger parts; the pinion differs by 3 % because the mass file rounds to 0.1 g. The other links include servos and bearings, so their volumes are not comparable one to one.

## The deviation, stated plainly

The spec asks for a model that is parametric **inside Fusion** and explicitly not made of imported bodies. This model is the opposite trade: exact detail, but the bodies have no feature history, and changing a User Parameter in Fusion does **not** change the geometry. To change the design: edit `cad/params/arm_params.json`, run `python tools/export_all.py`, run the script again.

Why: drawing all 21 parts feature by feature through the Fusion API would be a second hand-maintained copy of about 1,500 lines of CAD. A feature-based version at reduced detail exists as `build_native_skeleton.py` (simplified shapes, link lengths as live parameters); it has **not been run**.

If a model that is both full-detail and feature-parametric in Fusion is wanted, that is a separate piece of work.

## Notes from the first run

- The design must be created as a **Hybrid Design**; the script uses the open design if it is empty.
- The API cannot rename the root component (first run failed on that; fixed).
- A launcher in `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\build_exxeta_arm\` makes the script appear in the Scripts list; it only loads the real file from the project folder.
- Joint limits are not set in Fusion, and dragging was not tried.
