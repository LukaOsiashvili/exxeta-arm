"""Fusion 360 script: build the full-detail exxeta-arm assembly.

Role in the system
------------------
build123d (cad/parts/*.py) is the source of truth. This script gives the same
design to Fusion 360 users at FULL detail:

  * one component per link (base, turntable, upper_arm, forearm, wrist, gripper,
    pinion, finger_a, finger_b);
  * every component contains the exact part geometry (every pocket, hole, gear
    tooth), read from the per-link STEP files that tools/export_all.py writes to
    cad/export/fusion_links/. Printed parts and bought parts (servos, bearings)
    are separate sub-components, so the bought parts can be hidden;
  * joints J1..J5 (revolute), the two finger sliders and the pinion, so the arm
    can be dragged and posed in Fusion;
  * the main dimensions as User Parameters, for reference.

How it stays parametric: the geometry is regenerated, not edited. Change
cad/params/arm_params.json, run `python tools/export_all.py`, run this script
again. (A model drawn feature by feature in Fusion would be a second source of
truth to keep in step by hand; see docs/decisions/ADR-0009. A simplified
feature-based version exists in build_native_skeleton.py.)

Run: Utilities > Add-Ins > Scripts and Add-Ins > build_exxeta_arm > Run.
Output: a new document, cad/fusion360/fusion_report.json (volume and bounding box
per link, to compare with build123d) and cad/fusion360/exxeta_arm.f3d.
"""
import json
import os
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "..", "params", "arm_params.json")
LINKS_DIR = os.path.join(HERE, "..", "export", "fusion_links")
CM = 0.1                      # millimetres -> Fusion's internal centimetres

# (joint name, child link, parent link, kind, axis); the joint sits at the child link's origin
JOINTS = [
    ("J1 base_yaw", "turntable", "base", "revolute", "Z"),
    ("J2 shoulder_pitch", "upper_arm", "turntable", "revolute", "Y"),
    ("J3 elbow_pitch", "forearm", "upper_arm", "revolute", "Y"),
    ("J4 wrist_pitch", "wrist", "forearm", "revolute", "Y"),
    ("J5 wrist_roll", "gripper", "wrist", "revolute", "Z"),
    ("J6 pinion", "pinion", "gripper", "revolute", "Z"),
    ("J6 finger_a", "finger_a", "gripper", "slider", "X"),
    ("J6 finger_b", "finger_b", "gripper", "slider", "X"),
]


def strip(node):
    if isinstance(node, dict):
        return {k: strip(v) for k, v in node.items() if not k.startswith("_")}
    return node


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    log = []
    try:
        with open(PARAMS, encoding="utf-8") as f:
            P = strip(json.load(f))
        with open(os.path.join(LINKS_DIR, "index.json"), encoding="utf-8") as f:
            index = json.load(f)

        # Use the open design if it is still empty (create a "Hybrid Design" first: it may hold both
        # bodies and sub-components); otherwise start a new document.
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design or design.rootComponent.occurrences.count or design.rootComponent.bRepBodies.count:
            app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
            design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent          # (its name is the document name; the API cannot change it)

        # ---- User Parameters (Modify > Change Parameters), for reference ----
        g = P["geometry"]
        for name, value, comment in [
            ("upper_arm_length", g["upper_arm_length_mm"], "J2 axis to J3 axis"),
            ("forearm_length", g["forearm_length_mm"], "J3 axis to J4 axis"),
            ("wrist_to_tcp", g["wrist_to_tcp_mm"], "J4 axis to grip centre"),
            ("shoulder_height", g["shoulder_height_mm"], "table to J2 axis"),
            ("base_height", P["base"]["height_mm"], "height of the base shell"),
            ("link_half_width", P["joint_design"]["link_half_width"], "half width of every link"),
            ("gripper_opening_max", g["gripper_opening_max_mm"], "largest gap between the fingers"),
        ]:
            design.userParameters.add(name, adsk.core.ValueInput.createByString(f"{value} mm"), "mm",
                                      comment + " (from arm_params.json; change it there and re-run the script)")

        # ---- one component per link, filled with the exact STEP geometry ----
        occs = {}
        imp = app.importManager
        for link, entry in index.items():
            m = adsk.core.Matrix3D.create()
            x, y, z = entry["origin_mm"]
            m.translation = adsk.core.Vector3D.create(x * CM, y * CM, z * CM)
            occ = root.occurrences.addNewComponent(m)
            occ.component.name = link
            for fname in entry["files"]:
                opts = imp.createSTEPImportOptions(os.path.join(LINKS_DIR, fname))
                imp.importToTarget(opts, occ.component)
                child = occ.component.occurrences.item(occ.component.occurrences.count - 1)
                child.component.name = fname.replace(".step", "")
            occs[link] = occ
            log.append(f"{link}: {len(entry['files'])} file(s)")
        occs["base"].isGrounded = True

        # ---- joints ---------------------------------------------------------
        sketch = root.sketches.add(root.xYConstructionPlane)
        sketch.name = "joint origins"
        directions = {"X": adsk.fusion.JointDirections.XAxisJointDirection,
                      "Y": adsk.fusion.JointDirections.YAxisJointDirection,
                      "Z": adsk.fusion.JointDirections.ZAxisJointDirection}
        for name, child, parent, kind, axis in JOINTS:
            try:
                x, y, z = index[child]["origin_mm"]
                pt = sketch.sketchPoints.add(sketch.modelToSketchSpace(adsk.core.Point3D.create(x * CM, y * CM, z * CM)))
                geo = adsk.fusion.JointGeometry.createByPoint(pt)
                inp = root.asBuiltJoints.createInput(occs[child], occs[parent], geo)
                if kind == "revolute":
                    inp.setAsRevoluteJointMotion(directions[axis])
                else:
                    inp.setAsSliderJointMotion(directions[axis])
                root.asBuiltJoints.add(inp).name = name
            except Exception:
                log.append(f"joint '{name}' failed: {traceback.format_exc(limit=1).strip().splitlines()[-1]}")

        # ---- report: volume and bounding box per link -----------------------
        report = {"_about": "Written by build_exxeta_arm.py. Volumes include bought parts (servo and bearing dummies). "
                            "Compare with cad/export/masses.json and docs/design/cad_checks.md."}
        for link, occ in occs.items():
            try:
                bb = occ.boundingBox
                report[link] = {
                    "volume_cm3": round(occ.physicalProperties.volume, 3),
                    "bbox_mm": [round((bb.maxPoint.x - bb.minPoint.x) / CM, 2), round((bb.maxPoint.y - bb.minPoint.y) / CM, 2),
                                round((bb.maxPoint.z - bb.minPoint.z) / CM, 2)]}
            except Exception:
                report[link] = {"error": "could not measure"}
        report["log"] = log
        with open(os.path.join(HERE, "fusion_report.json"), "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)

        # ---- export ---------------------------------------------------------
        try:
            path = os.path.join(HERE, "exxeta_arm.f3d")
            design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(path))
            exported = "exported exxeta_arm.f3d"
        except Exception:
            exported = "f3d export failed (save the design first, then File > Export)"
        app.activeViewport.fit()
        ui.messageBox(f"exxeta-arm built: {len(occs)} components, {root.asBuiltJoints.count} joints.\n{exported}.\n\n" + "\n".join(log))
    except Exception:
        try:
            with open(os.path.join(HERE, "fusion_error.txt"), "w", encoding="utf-8") as f:
                f.write(traceback.format_exc())
        except Exception:
            pass
        ui.messageBox("build_exxeta_arm failed:\n" + traceback.format_exc())
