"""Fusion 360 script: build a native, parametric model of exxeta-arm.

Role in the system
------------------
build123d (cad/parts/*.py) is the source of truth for PRINTING. This script
builds a second, native Fusion 360 model from the SAME parameter file
(cad/params/arm_params.json), for people who prefer to explore or modify the
design in Fusion: real sketches and extrudes in a timeline, one component per
link, User Parameters, and joints you can drag.

What it is and is not (see docs/decisions/ADR-0009):
  * It models the main structure of every link: forks, bridges, servo housings
    with their pockets, base, turntable, gripper body and fingers.
  * It does NOT reproduce every detail of the printable parts (nut pockets,
    gear teeth, alignment marks, cable notches, the ball-race groove). Print
    from the STL files in cad/export/stl, not from this model.
  * The link lengths and base size are live User Parameters. Everything else is
    computed from arm_params.json when the script runs: change the JSON and run
    the script again to regenerate.

How to run: Fusion 360 > Utilities > Add-Ins > Scripts and Add-Ins > "+" > select
this file's folder > Run. It creates a new document, writes
cad/fusion360/fusion_report.json (volume and bounding box per component) and
exports cad/fusion360/exxeta_arm.f3d.

STATUS: written against the documented Fusion API but NOT yet run (it needs
Fusion 360). Expect to fix small things on the first run.
"""
import json
import os
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "..", "params", "arm_params.json")
MM = 0.1                      # Fusion's internal length unit is the centimetre

app = adsk.core.Application.get()
ui = app.userInterface


def strip(node):
    if isinstance(node, dict):
        return {k: strip(v) for k, v in node.items() if not k.startswith("_")}
    return node


class Builder:
    """Small helpers that create ordinary timeline features inside one component."""

    def __init__(self, design):
        self.design = design
        self.root = design.rootComponent

    def component(self, name, x=0.0, y=0.0, z=0.0):
        """New component placed at (x, y, z) mm in the assembly. Returns (occurrence, component)."""
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(x * MM, y * MM, z * MM)
        occ = self.root.occurrences.addNewComponent(m)
        occ.component.name = name
        return occ, occ.component

    def _extrude(self, comp, sketch, distance, op):
        prof = sketch.profiles.item(0)
        if sketch.profiles.count > 1:                     # take the largest profile (outer shape)
            prof = max(sketch.profiles, key=lambda p: p.areaProperties().area)
        ext = comp.features.extrudeFeatures
        inp = ext.createInput(prof, op)
        if isinstance(distance, str):
            dist = adsk.core.ValueInput.createByString(distance)      # an expression: stays parametric
        else:
            dist = adsk.core.ValueInput.createByReal(distance * MM)
        inp.setDistanceExtent(False, dist)
        return ext.add(inp)

    def _plane(self, comp, base, offset):
        planes = comp.constructionPlanes
        inp = planes.createInput()
        if isinstance(offset, str):
            inp.setByOffset(base, adsk.core.ValueInput.createByString(offset))
        else:
            inp.setByOffset(base, adsk.core.ValueInput.createByReal(offset * MM))
        return planes.add(inp)

    def box(self, comp, x0, x1, y0, y1, z0, height, cut=False):
        """Box from z0 upwards. z0 and height may be numbers (mm) or parameter expressions (strings)."""
        plane = self._plane(comp, comp.xYConstructionPlane, z0)
        sk = comp.sketches.add(plane)
        p0 = sk.modelToSketchSpace(adsk.core.Point3D.create(x0 * MM, y0 * MM, 0))
        p1 = sk.modelToSketchSpace(adsk.core.Point3D.create(x1 * MM, y1 * MM, 0))
        p0.z = p1.z = 0
        sk.sketchCurves.sketchLines.addTwoPointRectangle(p0, p1)
        op = adsk.fusion.FeatureOperations.CutFeatureOperation if cut else self._join(comp)
        return self._extrude(comp, sk, height, op)

    def cyl_z(self, comp, radius, x, y, z0, height, cut=False):
        plane = self._plane(comp, comp.xYConstructionPlane, z0)
        sk = comp.sketches.add(plane)
        c = sk.modelToSketchSpace(adsk.core.Point3D.create(x * MM, y * MM, 0))
        c.z = 0
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, radius * MM)
        op = adsk.fusion.FeatureOperations.CutFeatureOperation if cut else self._join(comp)
        return self._extrude(comp, sk, height, op)

    @staticmethod
    def _join(comp):
        ops = adsk.fusion.FeatureOperations
        return ops.JoinFeatureOperation if comp.bRepBodies.count else ops.NewBodyFeatureOperation


def run(context):
    try:
        with open(PARAMS, encoding="utf-8") as f:
            P = strip(json.load(f))
        g, jd, sm = P["geometry"], P["joint_design"], P["servo_mech"]
        m9, m90 = sm["MG996R"], sm["MG90S"]

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        b = Builder(design)
        up = design.userParameters

        def param(name, mm, comment):
            up.add(name, adsk.core.ValueInput.createByString(f"{mm} mm"), "mm", comment)

        # ---- User Parameters (Modify > Change Parameters) -----------------
        param("upper_arm_length", g["upper_arm_length_mm"], "J2 axis to J3 axis")
        param("forearm_length", g["forearm_length_mm"], "J3 axis to J4 axis")
        param("wrist_to_tcp", g["wrist_to_tcp_mm"], "J4 axis to grip centre")
        param("shoulder_height", g["shoulder_height_mm"], "table to J2 axis")
        param("base_height", P["base"]["height_mm"], "height of the base shell")
        param("link_half_width", jd["link_half_width"], "half width of every link")
        param("payload_g", P["design_targets"]["payload_g"], "rated payload in grams (information)")

        HW, wall = jd["link_half_width"], jd["wall"]
        L1, L2, L3 = g["upper_arm_length_mm"], g["forearm_length_mm"], g["wrist_to_tcp_mm"]
        SH, BH = g["shoulder_height_mm"], P["base"]["height_mm"]
        # joint stack along Y for the MG996R joints (same formulas as cad/common.py)
        total = jd["fork_horn_arm_t"] + m9["horn_face_h"] + 0.5 + jd["cradle_back_wall"] + jd["pivot_boss_h"] + 0.2 + jd["fork_bearing_arm_t"]
        W = total / 2
        y_horn = W - jd["fork_horn_arm_t"]
        y_bottom = y_horn - m9["horn_face_h"]
        y_ear = y_bottom + m9["bottom_to_ear"]
        y_wall_out = y_bottom - 0.5 - jd["cradle_back_wall"]
        z_near, z_far = -(m9["body_l"] - m9["shaft_offset"]), m9["shaft_offset"]   # servo body along the link, shaft at 0
        br, bt = jd["bridge_radius_large"], jd["bridge_t"]
        brg = P["hardware"]["bearing_pitch"]

        def cradle(comp, zc):
            """MG996R housing with its pocket; shaft axis along Y at height zc."""
            z0, z1 = zc + z_near - 0.25 - wall, zc + z_far + 0.25 + wall
            ya, yb = sorted((y_wall_out, y_ear))
            b.box(comp, -HW, HW, ya, yb, z0, z1 - z0)
            b.box(comp, -m9["body_w"] / 2 - 0.25, m9["body_w"] / 2 + 0.25, y_bottom - 0.5, yb + 0.01, zc + z_near - 0.25, m9["body_l"] + 0.5, cut=True)

        def fork(comp):
            """Fork around a housing at z = 0: two arms and the bridge."""
            top = br + bt
            b.box(comp, -HW, HW, y_horn, W, 0, top)                       # arm 1 (horn side)
            b.box(comp, -HW, HW, -W, -W + jd["fork_bearing_arm_t"], 0, top)   # arm 2 (bearing side)
            b.box(comp, -HW, HW, -W, W, br, bt)                           # bridge
            b.box(comp, -HW, HW, y_horn, W, -HW, HW)                      # rounded ends are simplified to square
            b.box(comp, -HW, HW, -W, -W + jd["fork_bearing_arm_t"], -HW, HW)

        # ---- base ----------------------------------------------------------
        fx, fy = P["base"]["footprint_mm"]
        bw = P["base"]["wall_mm"]
        occ_base, base = b.component("base")
        b.box(base, -fx / 2, fx / 2, -fy / 2, fy / 2, 0, "base_height")
        b.box(base, -fx / 2 + bw, fx / 2 - bw, -fy / 2 + bw, fy / 2 - bw, 0, "base_height - 3 mm", cut=True)
        tt = P["turntable"]
        r_ring = tt["race_pitch_radius"] + tt["race_gap"] / 2
        b.cyl_z(base, r_ring, 0, 0, BH - 3.5, 4.0, cut=True)              # opening for the turntable
        b.cyl_z(base, 56.0, 0, 0, BH, 6.0)                                # clamp ring
        b.cyl_z(base, r_ring, 0, 0, BH, 6.5, cut=True)

        # ---- turntable + shoulder tower ------------------------------------
        occ_tt, ttc = b.component("turntable", 0, 0, SH)
        r_disc = tt["race_pitch_radius"] - tt["race_gap"] / 2
        z_web_bot = BH - tt["rim_t"] / 2 - SH
        b.cyl_z(ttc, r_disc, 0, 0, z_web_bot, tt["web_t"])
        cradle(ttc, 0.0)
        z_box = z_near - 0.25 - wall
        z_web_top = z_web_bot + tt["web_t"]
        for x0, x1 in ((-HW, -HW + wall), (HW - wall, HW)):
            ya, yb = sorted((y_wall_out, y_ear))
            b.box(ttc, x0, x1, ya, yb, z_web_top, z_box - z_web_top)

        # ---- upper arm -----------------------------------------------------
        occ_ua, ua = b.component("upper_arm", 0, 0, SH)
        fork(ua)
        ya, yb = sorted((y_wall_out, y_ear))
        for x0, x1 in ((-HW, -HW + wall), (HW - wall, HW)):                # rails: their length follows the parameter
            b.box(ua, x0, x1, ya, yb, br + bt, f"upper_arm_length - {br + bt - z_box} mm")
        cradle(ua, L1)

        # ---- forearm -------------------------------------------------------
        occ_fa, fa = b.component("forearm", 0, 0, SH + L1)
        fork(fa)
        mic_total = jd["micro_horn_arm_t"] + (m90["horn_top_h"] - m90["horn_blade_t"]) + 0.4 + jd["cradle_back_wall"] + jd["pivot_boss_h"] + 0.2 + jd["fork_bearing_arm_t"]
        MW = mic_total / 2
        wbr = jd["bridge_radius_wrist"]
        b.box(fa, -HW, HW, -12, 12, br + bt, f"forearm_length - {br + bt + wbr + bt} mm")     # spine
        b.box(fa, -HW, HW, -MW, MW, L2 - wbr - bt, bt)                                          # wrist bridge
        b.box(fa, -HW, HW, MW - jd["micro_horn_arm_t"], MW, L2 - wbr, wbr + HW)                 # wrist fork arm 1
        b.box(fa, -HW, HW, -MW, -MW + jd["fork_bearing_arm_t"], L2 - wbr, wbr + HW)             # wrist fork arm 2

        # ---- wrist block (two MG90S) ---------------------------------------
        occ_wr, wr = b.component("wrist", 0, 0, SH + L1 + L2)
        over = (m90["ear_len"] - m90["body_l"]) / 2
        X0, X1 = -m90["shaft_offset"] - over - 0.6, m90["body_l"] - m90["shaft_offset"] + over + 0.6
        half = m90["body_w"] / 2 + 0.25 + jd["wall_thin"]
        y_arm1_in = MW - jd["micro_horn_arm_t"]
        yb4 = y_arm1_in - (m90["horn_top_h"] - m90["horn_blade_t"])
        b.box(wr, X0, X1, yb4 - 0.4 - jd["cradle_back_wall"], yb4 + m90["bottom_to_ear"], -half, 2 * half)
        b.box(wr, X0, X1, -half, half, half, m90["bottom_to_ear"])
        b.box(wr, -m90["shaft_offset"] - 0.25, m90["body_l"] - m90["shaft_offset"] + 0.25, -m90["body_w"] / 2 - 0.25, m90["body_w"] / 2 + 0.25,
              half, m90["bottom_to_ear"] + 0.1, cut=True)

        # ---- gripper body + fingers ----------------------------------------
        occ_gr, gr = b.component("gripper", 0, 0, SH + L1 + L2)
        z_plate0 = half + m90["boss_h"] + 1.6
        zb6 = half + m90["horn_top_h"] + 3.2
        rack_floor = zb6 + m90["boss_h"] + 0.05
        b.box(gr, -16, 22, -half, half, z_plate0, rack_floor - z_plate0)
        b.box(gr, -m90["shaft_offset"] - 0.25, m90["body_l"] - m90["shaft_offset"] + 0.25, -m90["body_w"] / 2 - 0.25, m90["body_w"] / 2 + 0.25,
              zb6, rack_floor - zb6 + 0.1, cut=True)
        pin_r = jd["gripper_pinion_module"] * jd["gripper_pinion_teeth"] / 2
        back = pin_r + 1.25 * jd["gripper_pinion_module"] + 3.0
        for s in (1, -1):                                                  # rack guide walls
            ya, yb = sorted((s * (back + 0.15), s * (back + 3.35)))
            b.box(gr, -16, 22, ya, yb, rack_floor - 8, 13)
        pad, ft, fh = jd["finger_pad_t"], jd["finger_t"], jd["finger_h"] / 2
        z_tip = L3 + g["tcp_to_fingertip_mm"]
        fingers = []
        for name, s in (("finger_a", 1), ("finger_b", -1)):
            occ_f, fc = b.component(name, 0, 0, SH + L1 + L2)
            xa, xb = sorted((s * pad, s * (pad + ft)))
            b.box(fc, xa, xb, -fh, fh, rack_floor + 5.3, z_tip - rack_floor - 5.3)          # finger
            ra, rb = sorted((s * -28.0, s * (pad + ft)))
            ya, yb = sorted((s * (pin_r + 1.25), s * back))
            b.box(fc, ra, rb, ya, yb, rack_floor + 0.2, 4.6)                                  # rack bar
            fingers.append(occ_f)

        # ---- joints: drag the arm in Fusion --------------------------------
        sk = b.root.sketches.add(b.root.xZConstructionPlane)

        def joint_point(x, y, z):
            p = sk.modelToSketchSpace(adsk.core.Point3D.create(x * MM, y * MM, z * MM))
            return sk.sketchPoints.add(p)

        JD_ = adsk.fusion.JointDirections

        def revolute(child, parent, z, axis, name):
            geo = adsk.fusion.JointGeometry.createByPoint(joint_point(0, 0, z))
            inp = b.root.asBuiltJoints.createInput(child, parent, geo)
            inp.setAsRevoluteJointMotion(axis)
            b.root.asBuiltJoints.add(inp).name = name

        occ_base.isGrounded = True
        revolute(occ_tt, occ_base, SH, JD_.ZAxisJointDirection, "J1 base_yaw")
        revolute(occ_ua, occ_tt, SH, JD_.YAxisJointDirection, "J2 shoulder_pitch")
        revolute(occ_fa, occ_ua, SH + L1, JD_.YAxisJointDirection, "J3 elbow_pitch")
        revolute(occ_wr, occ_fa, SH + L1 + L2, JD_.YAxisJointDirection, "J4 wrist_pitch")
        revolute(occ_gr, occ_wr, SH + L1 + L2 + half, JD_.ZAxisJointDirection, "J5 wrist_roll")
        for occ_f, nm in zip(fingers, ("J6 finger_a", "J6 finger_b")):
            geo = adsk.fusion.JointGeometry.createByPoint(joint_point(0, 0, SH + L1 + L2 + rack_floor))
            inp = b.root.asBuiltJoints.createInput(occ_f, occ_gr, geo)
            inp.setAsSliderJointMotion(JD_.XAxisJointDirection)
            b.root.asBuiltJoints.add(inp).name = nm

        # ---- report + export ------------------------------------------------
        report = {"_about": "Written by build_exxeta_arm.py: compare with cad/export/masses.json (see ADR-0009)."}
        for occ in b.root.occurrences:
            vol = sum(body.volume for body in occ.component.bRepBodies)            # cm^3
            bb = occ.boundingBox
            report[occ.component.name] = {
                "volume_cm3": round(vol, 2),
                "bbox_mm": [round((bb.maxPoint.x - bb.minPoint.x) / MM, 1), round((bb.maxPoint.y - bb.minPoint.y) / MM, 1),
                            round((bb.maxPoint.z - bb.minPoint.z) / MM, 1)]}
        with open(os.path.join(HERE, "fusion_report.json"), "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        try:
            opts = design.exportManager.createFusionArchiveExportOptions(os.path.join(HERE, "exxeta_arm.f3d"))
            design.exportManager.execute(opts)
            exported = "exported exxeta_arm.f3d"
        except Exception:
            exported = "f3d export failed (save the design to the cloud first, then File > Export)"
        app.activeViewport.fit()
        ui.messageBox(f"exxeta-arm built from arm_params.json.\n{len(report) - 1} components, report written, {exported}.")
    except Exception:
        ui.messageBox("build_exxeta_arm failed:\n" + traceback.format_exc())
