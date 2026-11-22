"""Assembly of exxeta-arm: puts every part in place for a given set of joint angles.

Role in the system
------------------
Used by tools/export_all.py for the STEP assembly, the renders, the interference
check and the URDF meshes. It is also the reference for how the link frames
relate to each other (the kinematics code and the URDF use exactly this chain):

    world -> J1 (yaw about Z, at shoulder height) -> turntable frame
          -> J2 (pitch about Y)                    -> upper arm frame
          -> +Z upper_arm_length, J3 (pitch)       -> forearm frame
          -> +Z forearm_length,  J4 (pitch)        -> wrist frame
          -> J5 (roll about Z)                     -> gripper frame
          -> J6 (servo angle; each finger slides +-pinion_radius x angle along X) -> fingers

Run directly to print the part list:  python cad/assembly.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from build123d import Location, Pos, Rot  # noqa: E402

from common import (BRG, HAND, JD, LARGE, MICRO, P, TT, bearing_dummy, mg90s_dummy, mg996r_dummy)  # noqa: E402
from parts import all_parts  # noqa: E402

# Named poses, joint angles in degrees (J1..J6). See joints._about in arm_params.json.
POSES = {
    "zero": (0, 0, 0, 0, 0, 0),                 # straight up
    "home": (0, 20, 70, 0, 0, 90),              # relaxed "ready" pose
    "reach": (0, 90, 0, 0, 0, 90),              # horizontal, fully stretched: the torque worst case
    "rest": (0, -72, 120, 85, 0, 0),            # parked: folded, leaning on the J2 back stop
}


def link_frames(q) -> dict[str, Location]:
    """World location of every link frame for joint angles q = (J1..J6) in degrees."""
    q1, q2, q3, q4, q5, q6 = q
    g = P.geometry
    tt = Pos(0, 0, g.shoulder_height_mm) * Rot(0, 0, q1)
    ua = tt * Pos(g.shoulder_offset_mm, 0, 0) * Rot(0, q2, 0)
    fa = ua * Pos(0, 0, g.upper_arm_length_mm) * Rot(0, q3, 0)
    wr = fa * Pos(0, 0, g.forearm_length_mm) * Rot(0, q4, 0)
    gr = wr * Rot(0, 0, q5)
    d = math.radians(q6) * HAND.pin_r          # travel of each finger
    return {"base": Location(), "turntable": tt, "upper_arm": ua, "forearm": fa,
            "wrist": wr, "gripper": gr,
            "pinion": gr * Rot(0, 0, -q6),
            "finger_a": gr * Pos(d, 0, 0),
            "finger_b": gr * Pos(-d, 0, 0) * Rot(0, 0, 180)}


def part_instances():
    """[(PartDef, link, local Location)] for every printed part instance of the arm itself."""
    g = P.geometry
    out = []
    for pd in all_parts():
        if pd.name in ("tolerance_test", "test_rig_stand", "test_rig_arm", "pot_pod", "pot_coupler",
                       "j1_pot_gear", "pivot_cover"):
            continue  # not part of the standard (no-pot) assembly view
        if pd.name == "pivot_pin":
            out.append((pd, "turntable", Pos(0, LARGE.y_boss - BRG.width_mm - JD.pivot_pin_flange_t, 0)))
            out.append((pd, "upper_arm", Pos(0, LARGE.y_boss - BRG.width_mm - JD.pivot_pin_flange_t, g.upper_arm_length_mm)))
            out.append((pd, "wrist", Pos(0, MICRO.y_boss - BRG.width_mm - JD.pivot_pin_flange_t, 0)))
        elif pd.name in ("fingertip", "gripper_finger"):
            out.append((pd, "finger_a", Location()))
            out.append((pd, "finger_b", Location()))
        else:
            out.append((pd, pd.link, Location()))
    return out


def vendor_instances():
    """[(name, link, solid in link frame)] for bought parts: servos and bearings (simplified shapes)."""
    g, h = P.geometry, HAND
    m996 = P.servo_mech.MG996R
    out = [
        ("J1 servo", "base", Pos(0, 0, TT.z_bracket_top - m996.bottom_to_ear) * Rot(0, 0, -90) * Rot(90, 0, 0) * mg996r_dummy()),
        ("J2 servo", "turntable", Pos(0, LARGE.y_bottom, 0) * mg996r_dummy()),
        ("J2 bearing", "turntable", Pos(0, LARGE.y_boss - BRG.width_mm / 2, 0) * bearing_dummy()),
        ("J3 servo", "upper_arm", Pos(0, LARGE.y_bottom, g.upper_arm_length_mm) * mg996r_dummy()),
        ("J3 bearing", "upper_arm", Pos(0, LARGE.y_boss - BRG.width_mm / 2, g.upper_arm_length_mm) * bearing_dummy()),
        ("J4 servo", "wrist", Pos(0, MICRO.y_bottom, 0) * mg90s_dummy(with_horn=False)),
        ("J4 bearing", "wrist", Pos(0, MICRO.y_boss - BRG.width_mm / 2, 0) * bearing_dummy()),
        ("J5 servo", "wrist", Pos(0, 0, h.z0) * Rot(90, 0, 0) * mg90s_dummy(with_horn=False)),
        ("J6 servo", "gripper", Pos(0, 0, h.zb6) * Rot(90, 0, 0) * mg90s_dummy(with_horn=False)),
    ]
    return out


def build(q=POSES["home"], with_vendor=True):
    """Return [(label, kind, link, color, solid in world frame)] for the whole arm at pose q."""
    frames = link_frames(q)
    cache = {}
    items = []
    for pd, link, loc in part_instances():
        if pd.name not in cache:
            cache[pd.name] = pd.make()
        items.append((pd.name, "printed", link, pd.color, frames[link] * loc * cache[pd.name]))
    if with_vendor:
        for name, link, solid in vendor_instances():
            items.append((name, "vendor", link, "vendor", frames[link] * solid))
    return items


if __name__ == "__main__":
    for pd in all_parts():
        print(f"{pd.number:02d} {pd.name:16s} x{pd.qty}  link={pd.link}")
