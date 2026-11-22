"""Shared CAD helpers for exxeta-arm (build123d).

Role in the system
------------------
Every part module in cad/parts/ imports this file. It provides
  * `P`  : the parameters from cad/params/arm_params.json (dot access),
  * small shape helpers (`box`, `cyl_x/y/z`, `hex_prism`, ...) that take
    explicit coordinates, so part code reads like a dimensioned drawing,
  * the two joint "stacks" (`LARGE` for MG996R joints, `MICRO` for the MG90S
    wrist joint): where every face sits along the joint axis,
  * reusable building blocks: servo cradle, fork, servo dummies.

Conventions
-----------
* Units: millimetres. Angles: degrees.
* Link frames (same as the URDF): origin on the link's proximal joint axis,
  +Z along the link towards the next joint (arm pointing up = zero pose),
  +Y along the pitch-joint axes (the servo horn is on the +Y side),
  +X forward.
* Printing: every arm link is designed to lie on its flat -X face.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

from build123d import (Align, Axis, Box, Cylinder, Plane, Polygon, Pos, RectangleRounded, RegularPolygon, Rot,
                       Circle, Rectangle, extrude, offset, revolve, Part, Sketch)

CAD_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CAD_DIR.parent
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
from arm_params import load_params  # noqa: E402  (shared loader, strips comments)


def _ns(node):
    """dict -> SimpleNamespace, recursively, for P.geometry.upper_arm_length_mm style access."""
    if isinstance(node, dict):
        return SimpleNamespace(**{k: _ns(v) for k, v in node.items()})
    if isinstance(node, list):
        return [_ns(v) for v in node]
    return node


RAW = load_params()
P = _ns(RAW)
TOL = P.tolerances
JD = P.joint_design
HW = JD.link_half_width  # half width of every link (X direction)

# Fastener constants (ISO metric M3)
M3_CLEAR = 3.0 + 2 * TOL.hole_clearance_mm        # hole a screw passes through freely
M3_TAP = 2.6                                      # hole an M3 screw cuts its own thread into (PETG)
M3_HEAD_D = 6.2                                   # counterbore for cap/pan head
M3_NUT_AF = 5.5 + 2 * TOL.nut_pocket_clearance_mm  # across flats
M3_NUT_T = 2.4 + 0.2
M2_PILOT = 1.7                                    # servo self-tapping screws bite into this

EPS = 0.01


# --------------------------------------------------------------------------
# Shape helpers with explicit coordinates
# --------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))


def cyl_z(r, z0, z1, x=0.0, y=0.0) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(r, abs(z1 - z0))


def cyl_y(r, y0, y1, x=0.0, z=0.0) -> Part:
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(r, abs(y1 - y0))


def cyl_x(r, x0, x1, y=0.0, z=0.0) -> Part:
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(r, abs(x1 - x0))


def hex_prism_y(af, y0, y1, x=0.0, z=0.0) -> Part:
    """Hexagonal prism along Y (nut pocket). `af` = across flats."""
    h = abs(y1 - y0)
    solid = extrude(RegularPolygon(af / 2, 6, major_radius=False), h)
    return Pos(x, max(y0, y1), z) * Rot(90, 0, 0) * solid


def hex_prism_z(af, z0, z1, x=0.0, y=0.0) -> Part:
    return Pos(x, y, min(z0, z1)) * extrude(RegularPolygon(af / 2, 6, major_radius=False), abs(z1 - z0))


def prism_xz(points, y0, y1) -> Part:
    """Extrude a polygon drawn in the XZ plane (list of (x, z)) from y0 to y1."""
    sk = Plane.XZ * Polygon(*points, align=None)
    solid = extrude(sk, abs(y1 - y0))          # Plane.XZ normal is -Y: goes from 0 to -h
    return Pos(0, max(y0, y1), 0) * solid


def sketch_prism_xz(sketch: Sketch, y0, y1) -> Part:
    """Extrude a 2D sketch (drawn as x, z) from y0 to y1."""
    solid = extrude(Plane.XZ * sketch, abs(y1 - y0))
    return Pos(0, max(y0, y1), 0) * solid


def ring_z(r_out, r_in, z0, z1, x=0.0, y=0.0) -> Part:
    return cyl_z(r_out, z0, z1, x, y) - cyl_z(r_in, z0 - 1, z1 + 1, x, y)


def revolve_rz(points) -> Part:
    """Revolve a polygon given as (r, z) points around the Z axis."""
    return revolve(Plane.XZ * Polygon(*points, align=None), Axis.Z, 360)


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def union(*solids) -> Part:
    out = solids[0]
    for s in solids[1:]:
        out = out + s
    return out


# --------------------------------------------------------------------------
# Joint stacks: where each face sits along the joint axis (Y)
# --------------------------------------------------------------------------
class _LargeStack:
    """MG996R pitch joint (J2, J3). Symmetric about y = 0: outer faces at +-W.

    From +Y to -Y:  fork arm 1 | metal horn | servo (shaft up) | 0.5 gap |
                    back wall | pivot boss | 0.2 gap | fork arm 2 with 608 bearing
    """

    def __init__(self):
        m = P.servo_mech.MG996R
        self.m = m
        self.arm1_t = JD.fork_horn_arm_t
        self.arm2_t = JD.fork_bearing_arm_t
        total = (self.arm1_t + m.horn_face_h + 0.5 + JD.cradle_back_wall + JD.pivot_boss_h + 0.2 + self.arm2_t)
        self.W = total / 2
        self.y_horn = self.W - self.arm1_t           # horn outer face = inner face of fork arm 1
        self.y_bottom = self.y_horn - m.horn_face_h  # bottom of the servo case
        self.y_ear = self.y_bottom + m.bottom_to_ear  # underside of the mounting ears
        self.y_wall_in = self.y_bottom - 0.5
        self.y_wall_out = self.y_wall_in - JD.cradle_back_wall
        self.y_boss = self.y_wall_out - JD.pivot_boss_h
        self.y_arm2_in = self.y_boss - 0.2
        self.y_arm2_out = -self.W
        # extents of the servo body along the link (Z), shaft axis at z = 0
        self.z_far = m.shaft_offset                   # body end beyond the axis
        self.z_near = -(m.body_l - m.shaft_offset)    # body end towards the proximal joint
        self.ear_over = (m.ear_len - m.body_l) / 2    # how far each ear sticks out
        self.bridge_r = JD.bridge_radius_large


class _MicroStack:
    """MG90S wrist pitch joint (J4). The servo sits in the wrist block (distal link),
    the fork belongs to the forearm (ADR-0005)."""

    def __init__(self):
        m = P.servo_mech.MG90S
        self.m = m
        self.arm1_t = JD.micro_horn_arm_t
        self.arm2_t = JD.fork_bearing_arm_t
        horn_seat = m.horn_top_h - m.horn_blade_t   # inner face of arm 1 (horn blades sit in a pocket)
        total = self.arm1_t + horn_seat + 0.4 + JD.cradle_back_wall + JD.pivot_boss_h + 0.2 + self.arm2_t
        self.W = total / 2
        self.y_arm1_in = self.W - self.arm1_t
        self.y_bottom = self.y_arm1_in - horn_seat
        self.y_ear = self.y_bottom + m.bottom_to_ear
        self.y_wall_in = self.y_bottom - 0.4
        self.y_wall_out = self.y_wall_in - JD.cradle_back_wall
        self.y_boss = self.y_wall_out - JD.pivot_boss_h
        self.y_arm2_in = self.y_boss - 0.2
        self.y_arm2_out = -self.W
        self.bridge_r = JD.bridge_radius_wrist


class _Turntable:
    """Heights (world Z, table = 0) and ball-race geometry of the J1 turntable."""

    def __init__(self):
        t, m = P.turntable, P.servo_mech.MG996R
        self.z_bc = P.base.height_mm              # ball centre plane = top of the base deck
        self.rp = t.race_pitch_radius
        self.r_disc = self.rp - t.race_gap / 2    # turntable outer radius
        self.r_ring = self.rp + t.race_gap / 2    # inner radius of the outer rings
        # distance from the ball centre to the tip of each 90-degree V-groove
        self.a = (t.ball_d / 2 + t.race_clearance) * math.sqrt(2)
        self.z_web_bot = self.z_bc - t.rim_t / 2  # underside of the turntable = top face of the J1 horn
        self.z_web_top = self.z_web_bot + t.web_t
        self.z_rim_top = self.z_bc + t.rim_t / 2
        self.z_bracket_top = self.z_web_bot - (m.horn_face_h - m.bottom_to_ear)  # J1 servo ears rest here
        self.bracket_t = 4.0
        self.z_shoulder = self.z_web_top + JD.shoulder_axis_above_turntable
        if abs(self.z_shoulder - P.geometry.shoulder_height_mm) > 1e-6:
            raise ValueError(f"geometry.shoulder_height_mm must be {self.z_shoulder} for this base "
                             f"(base height + turntable + tower). Update arm_params.json.")
        self.gear_pd = t.pot_gear_module * t.pot_gear_teeth


TT = _Turntable()
LARGE = _LargeStack()
MICRO = _MicroStack()
BRG = P.hardware.bearing_pitch


# --------------------------------------------------------------------------
# Reusable features
# --------------------------------------------------------------------------
def nut_trap_y(x, y_open, depth, z) -> Part:
    """Hex recess for an M3 nut, opening at y = y_open, going `depth` towards +Y (depth>0) or -Y (depth<0)."""
    return hex_prism_y(M3_NUT_AF, y_open, y_open + depth, x, z)


def micro_horn_sketch(clearance=0.2) -> Sketch:
    """Outline of the MG90S double-arm horn, long axis along sketch-Y (= link Z after sketch_prism_xz)."""
    m = P.servo_mech.MG90S
    L = m.horn_len / 2 - m.horn_tip_w / 2
    pts = [(-m.horn_tip_w / 2, -L), (-m.horn_root_w / 2, 0), (-m.horn_tip_w / 2, L),
           (m.horn_tip_w / 2, L), (m.horn_root_w / 2, 0), (m.horn_tip_w / 2, -L)]
    sk = (Polygon(*pts, align=None) + Pos(0, L) * Circle(m.horn_tip_w / 2) + Pos(0, -L) * Circle(m.horn_tip_w / 2)
          + Circle(m.horn_hub_d / 2))
    return offset(sk, clearance)


def trimmed_horn_sketch(clearance=0.2) -> Sketch:
    """Outline of an MG90S double-arm horn cut down to JD.horn_trim_len, long axis along sketch-X."""
    m = P.servo_mech.MG90S
    sk = Rectangle(JD.horn_trim_len, m.horn_root_w) + Circle(m.horn_hub_d / 2)
    return offset(sk, clearance)


def rbox_y(x0, x1, y0, y1, z0, z1, r=3.0) -> Part:
    """Box with its four edges parallel to Y rounded (soft look, still flat on the print face)."""
    sk = RectangleRounded(abs(x1 - x0), abs(z1 - z0), r)
    solid = extrude(Plane.XZ * sk, abs(y1 - y0))
    return Pos((x0 + x1) / 2, max(y0, y1), (z0 + z1) / 2) * solid


def rbox_z(x0, x1, y0, y1, z0, z1, r=3.0) -> Part:
    """Box with its four vertical (Z) edges rounded."""
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, min(z0, z1)) * extrude(RectangleRounded(abs(x1 - x0), abs(y1 - y0), r), abs(z1 - z0))


def rbox_x(x0, x1, y0, y1, z0, z1, r=3.0) -> Part:
    """Box with its four edges parallel to X rounded."""
    solid = extrude(Plane.YZ * RectangleRounded(abs(y1 - y0), abs(z1 - z0), r), abs(x1 - x0))
    return Pos(min(x0, x1), (y0 + y1) / 2, (z0 + z1) / 2) * solid


def rack_sketch(module, x0, x1, tip_y, root_y, back_y, backlash=0.15) -> Sketch:
    """2D outline (x, y) of a gear rack with teeth pointing towards -Y (towards the pinion at the origin).

    A tooth is centred on x = 0, which matches gear_sketch() for an even tooth count
    (the pinion has a gap pointing straight at the rack).
    """
    pitch = math.pi * module
    t20 = math.tan(math.radians(20))
    pitch_y = tip_y + module
    half = pitch / 4 - backlash / 2
    pts = [(x0, back_y), (x0, root_y)]
    k0, k1 = math.ceil((x0 + 2.0) / pitch), math.floor((x1 - 2.0) / pitch)
    for k in range(k0, k1 + 1):
        xc = k * pitch
        pts += [(xc - half - (root_y - pitch_y) * t20, root_y), (xc - half + module * t20, tip_y),
                (xc + half - module * t20, tip_y), (xc + half + (root_y - pitch_y) * t20, root_y)]
    pts += [(x1, root_y), (x1, back_y)]
    return Polygon(*pts, align=None)


def pivot_seat_cut(y_wall_out, y_boss, y_wall_in) -> Part:
    """Cuts for the pivot pin: centring socket in the boss, M3 hole, nut pocket inside the back wall."""
    socket = cyl_y(BRG.id_mm / 2 + 0.1, y_boss - EPS, y_boss + 2.0)
    hole = cyl_y(M3_CLEAR / 2, y_boss - 1, y_wall_in + 1)
    nut = nut_trap_y(0, y_wall_in + EPS, -M3_NUT_T, 0)
    return socket + hole + nut


def mg996r_cradle():
    """Housing for an MG996R whose output shaft is the joint axis (Y through the origin).

    The servo body extends towards -Z. Returns (solid_to_add, solid_to_cut).
    Layout along Y comes from LARGE. The servo drops in from +Y and its ears are
    screwed (M3 + nut) to two ledges. The back wall carries the pivot boss for the
    608 bearing on the opposite side of the horn.
    """
    s, m, w = LARGE, LARGE.m, JD.wall
    c = TOL.servo_pocket_clearance_mm
    z0, z1 = s.z_near - c - w, s.z_far + c + w          # outer box along the link
    ledge_t = 4.5
    add = rbox_y(-HW, HW, s.y_wall_out, s.y_ear, z0, z1, 2.5)
    # ear ledges (plates in the XZ plane, sticking out at both ends)
    add += box(-HW, HW, s.y_ear - ledge_t, s.y_ear, s.z_near - s.ear_over - 0.5, s.z_far + s.ear_over + 0.5)
    # pivot boss on the back wall
    add += cyl_y(6.0, s.y_boss, s.y_wall_out)

    cut = box(-m.body_w / 2 - c, m.body_w / 2 + c, s.y_wall_in, s.y_ear + 30, s.z_near - c, s.z_far + c)
    # ear screw holes + nut recesses under the ledges
    zc = (s.z_near + s.z_far) / 2
    for dz in (-m.ear_hole_spacing_l / 2, m.ear_hole_spacing_l / 2):
        for dx in (-m.ear_hole_spacing_w / 2, m.ear_hole_spacing_w / 2):
            cut += cyl_y(M3_CLEAR / 2, s.y_ear - ledge_t - 6, s.y_ear + 1, dx, zc + dz)
            cut += nut_trap_y(dx, s.y_ear - ledge_t - 6, 6 + 2.2, zc + dz)
    # cable exit notches at both ends of the pocket (clones differ on which end the cable leaves)
    cut += box(-4.5, 4.5, s.y_wall_in, s.y_wall_in + 9, z0 - 1, z1 + 1)
    cut += pivot_seat_cut(s.y_wall_out, s.y_boss, s.y_wall_in)
    # alignment index line on the back wall, pointing away from the axis (see fork marks)
    cut += box(-0.6, 0.6, s.y_wall_out - EPS, s.y_wall_out + 0.6, z0 + 3, -HW - 0.8)
    return add, cut


def fork_marks(y_face, depth_dir, base_deg) -> Part:
    """Three short radial grooves on a fork-arm face: at base_deg and +-90 deg from it.

    They line up with the index line on the servo housing at joint angles 0, +90, -90.
    Angles are measured in the XZ plane from +Z towards +X.
    """
    r0, r1 = BRG.od_mm / 2 + 0.4, HW - 0.3
    cut = None
    for d in (base_deg, base_deg + 90, base_deg - 90):
        g = box(-0.6, 0.6, y_face, y_face + depth_dir * 0.6, r0, r1)
        g = Rot(0, d, 0) * g
        cut = g if cut is None else cut + g
    return cut


def bearing_arm(y_in, y_out, z_to, pod_z) -> tuple[Part, Part]:
    """Fork arm 2: plate with the 608 seat. y_in = face towards the housing, y_out = outer face.

    The seat is a slip fit and deeper than the bearing, so the bearing can float
    axially and absorb differences in horn height between servo clones.
    `z_to` = where the arm ends (the bridge); `pod_z` = z of the two pot-pod / cover screws.
    """
    lo, hi = min(0, z_to), max(0, z_to)
    add = cyl_y(HW, y_out, y_in) + box(-HW, HW, y_out, y_in, lo, hi)
    seat_r = BRG.od_mm / 2 + TOL.bearing_seat_clearance_mm
    lip = 0.6
    cut = cyl_y(seat_r, y_out + lip, y_in + EPS)            # seat, open towards the housing
    cut += cyl_y(BRG.od_mm / 2 - 1.3, y_out - 1, y_in)      # through hole inside the retaining lip
    for dx in (-7.0, 7.0):                                  # pot pod / cover screws with captive nuts
        cut += cyl_y(M3_CLEAR / 2, y_out - 1, y_in + 1, dx, pod_z)
        cut += nut_trap_y(dx, y_in + EPS, -M3_NUT_T, pod_z)
    return add, cut


def large_fork(link_dir=+1):
    """U-shaped fork that straddles an MG996R cradle. Joint axis = Y through the origin.

    link_dir=+1: the link continues towards +Z (fork at the proximal end of a link).
    Returns (add, cut). Arm 1 (+Y) screws to the metal horn, arm 2 (-Y) holds the bearing.
    """
    s = LARGE
    zb0 = link_dir * s.bridge_r
    zb1 = link_dir * (s.bridge_r + JD.bridge_t)
    lo, hi = min(0, zb1), max(0, zb1)
    # arm 1: horn side
    add = cyl_y(HW, s.y_horn, s.W) + box(-HW, HW, s.y_horn, s.W, lo, hi)
    cut = cyl_y(4.2, s.y_horn - 1, s.W + 1)                       # access to the horn's centre screw
    for k in range(4):                                            # 4 x M3 into the threaded horn
        x, z = polar(s.m.horn_pcd / 2, 45 + 90 * k)
        cut += cyl_y(M3_CLEAR / 2, s.y_horn - 1, s.W + 1, x, z)
    # arm 2: bearing side
    a2, c2 = bearing_arm(s.y_arm2_in, s.y_arm2_out, zb1, link_dir * 17.0)
    add += a2
    cut += c2
    cut += fork_marks(s.y_arm2_out - EPS, +1, 180 if link_dir > 0 else 0)
    # bridge joining the two arms
    add += rbox_x(-HW, HW, -s.W, s.W, min(zb0, zb1), max(zb0, zb1), 1.5)
    return add, cut


# --------------------------------------------------------------------------
# Dummy models of bought parts (assembly views, interference checks)
# --------------------------------------------------------------------------
def mg996r_dummy(with_horn=True) -> Part:
    """MG996R in its own frame: shaft along +Y through the origin, case bottom at y = 0, body towards -Z."""
    m = P.servo_mech.MG996R
    z1, z0 = m.shaft_offset, -(m.body_l - m.shaft_offset)
    over = (m.ear_len - m.body_l) / 2
    s = box(-m.body_w / 2, m.body_w / 2, 0, m.case_h, z0, z1)
    s += box(-m.body_w / 2, m.body_w / 2, m.bottom_to_ear, m.bottom_to_ear + m.ear_t, z0 - over, z1 + over)
    s += cyl_y(3.0, m.case_h, m.horn_face_h - 1.5)
    if with_horn:
        s += cyl_y(m.horn_d / 2, m.horn_face_h - m.horn_t, m.horn_face_h)
    return s


def mg90s_dummy(with_horn=True) -> Part:
    """MG90S in its own frame: shaft along +Y through the origin, case bottom at y = 0, body towards +X."""
    m = P.servo_mech.MG90S
    x0, x1 = -m.shaft_offset, m.body_l - m.shaft_offset
    over = (m.ear_len - m.body_l) / 2
    s = box(x0, x1, 0, m.case_h, -m.body_w / 2, m.body_w / 2)
    s += box(x0 - over, x1 + over, m.bottom_to_ear, m.bottom_to_ear + m.ear_t, -m.body_w / 2, m.body_w / 2)
    s += cyl_y(m.boss_d / 2, m.case_h, m.boss_h)
    s += cyl_y(2.4, m.boss_h, m.horn_top_h - 1.0)
    if with_horn:
        s += sketch_prism_xz(micro_horn_sketch(0.0), m.horn_top_h - m.horn_blade_t, m.horn_top_h)
    return s


def bearing_dummy() -> Part:
    """608 bearing, axis along Y, centred on the origin."""
    return cyl_y(BRG.od_mm / 2, -BRG.width_mm / 2, BRG.width_mm / 2) - cyl_y(BRG.id_mm / 2, -5, 5)


# --------------------------------------------------------------------------
# Part registry
# --------------------------------------------------------------------------
class PartDef:
    """Describes one printed part: how to build it and how to print it."""

    def __init__(self, number, name, make, qty=1, link="base", color="body", optional=False,
                 material="PETG", orientation="", supports="none", notes=""):
        self.number, self.name, self.make, self.qty = number, name, make, qty
        self.link = link            # which moving body it belongs to (assembly + mass model)
        self.color = color          # "body" or "accent" (two-colour look)
        self.optional = optional    # only needed with pots, or a test/calibration aid
        self.material, self.orientation, self.supports, self.notes = material, orientation, supports, notes

    @property
    def filename(self) -> str:
        return f"{self.number:02d}_{self.name}_{self.qty}x.stl"


def mass_g(part: Part) -> float:
    pr = P.mass_model.print
    return part.volume / 1000.0 * pr.density_g_cm3 * pr.fill_factor


def gear_sketch(module: float, teeth: int, backlash: float = 0.15, pressure_deg: float = 20.0, steps: int = 6) -> Sketch:
    """2D outline of an involute spur gear (used for the 1:1 gear pair of the optional J1 pot).

    `backlash` thins every tooth a little so two printed gears mesh without binding.
    """
    pa = math.radians(pressure_deg)
    rp = module * teeth / 2          # pitch radius
    rb = rp * math.cos(pa)           # base circle (where the involute starts)
    ra = rp + module                 # tip radius
    rf = rp - 1.25 * module          # root radius
    inv = lambda a: math.tan(a) - a  # noqa: E731  involute function
    half = math.pi / (2 * teeth) - backlash / (2 * rp)   # half tooth angle on the pitch circle
    r_start = max(rb, rf)
    flank = []
    for k in range(steps + 1):
        r = r_start + (ra - r_start) * k / steps
        alpha = math.acos(min(1.0, rb / r))
        flank.append((r, half + inv(pa) - inv(alpha)))   # (radius, angle from the tooth centre line)
    pts = []
    for i in range(teeth):
        c = 2 * math.pi * i / teeth
        pts.append((rf * math.cos(c - flank[0][1]), rf * math.sin(c - flank[0][1])))
        pts += [(r * math.cos(c - t), r * math.sin(c - t)) for r, t in flank]
        pts += [(r * math.cos(c + t), r * math.sin(c + t)) for r, t in reversed(flank)]
        pts.append((rf * math.cos(c + flank[0][1]), rf * math.sin(c + flank[0][1])))
    return Polygon(*pts, align=None)


class _Hand:
    """Shared geometry of wrist block, wrist cap, gripper body and jaw (all in the wrist frame:
    origin on the J4 axis, +Z along the hand = J5 roll axis)."""

    def __init__(self):
        m = P.servo_mech.MG90S
        c, w = TOL.servo_pocket_clearance_mm, JD.wall_thin
        self.m, self.c, self.w = m, c, w
        self.over = (m.ear_len - m.body_l) / 2
        # J4 and J5 servo bodies both run along +X with the shaft at x = 0
        self.bx0, self.bx1 = -m.shaft_offset, m.body_l - m.shaft_offset
        self.xc = (self.bx0 + self.bx1) / 2
        self.X0, self.X1 = self.bx0 - self.over - 0.6, self.bx1 + self.over + 0.6
        self.half = m.body_w / 2 + c + w            # half thickness of a pocket incl. walls
        self.z0 = self.half                          # bottom of the J5 (roll) servo
        self.z_ear = self.z0 + m.bottom_to_ear       # J5 ears rest here
        self.z_ear_top = self.z_ear + m.ear_t
        self.z_case = self.z0 + m.case_h + 0.3
        self.z_cap_top = self.z_case + 1.5           # top of the cap plate
        self.z_journal_top = self.z0 + m.boss_h + 1.4
        self.journal_r = 12.0
        self.z_plate0 = self.z_journal_top + 0.2     # gripper base plate
        self.z_plate1 = self.z0 + m.horn_top_h
        self.z_recess = self.z_plate1 - m.horn_blade_t - 0.1
        # J6 (gripper) servo: stands on the roll axis, shaft along +Z, body along +X like J5.
        # Rack-and-pinion parallel gripper (ADR-0006): a pinion on the J6 horn drives two racks
        # in opposite directions; each rack carries one finger.
        self.zb6 = self.z_plate1 + 3.2               # bottom of the J6 servo (clears the J5 horn screw)
        self.z6_ear = self.zb6 + m.bottom_to_ear
        self.z6_boss = self.zb6 + m.boss_h
        self.z6_horn_top = self.zb6 + m.horn_top_h
        self.pin_mod, self.pin_teeth = JD.gripper_pinion_module, JD.gripper_pinion_teeth
        self.pin_r = self.pin_mod * self.pin_teeth / 2          # pitch radius: finger travel per radian
        self.pin_z0 = self.z6_boss + 0.3                        # pinion sits on the servo boss ...
        self.pin_z1 = self.z6_horn_top                          # ... and is clamped by the horn on top
        self.rack_floor = self.pin_z0 - 0.25
        self.rack_z0, self.rack_z1 = self.pin_z0 - 0.05, self.pin_z1 - 0.05
        self.rack_tip_y = self.pin_r - self.pin_mod             # tooth tips
        self.rack_root_y = self.pin_r + 1.25 * self.pin_mod     # tooth roots
        self.rack_back_y = self.rack_root_y + 3.0
        self.guide_x0, self.guide_x1 = -16.0, 22.0              # body length along the rack travel
        self.wall_y0 = self.rack_back_y + 0.15
        self.wall_y1 = self.wall_y0 + 3.2
        self.travel = math.radians(P.joints.J6.max_deg) * self.pin_r   # travel of each finger
        # fingers (closed position): grip faces meet at x = 0
        self.pad_t = JD.finger_pad_t
        self.finger_t = JD.finger_t
        self.fy = JD.finger_h / 2
        self.z_tcp = P.geometry.wrist_to_tcp_mm
        self.z_tip = self.z_tcp + P.geometry.tcp_to_fingertip_mm
        self.z_finger0 = self.rack_z1 + 0.6
        self.rack_x0, self.rack_x1 = -28.0, self.pad_t + self.finger_t


HAND = _Hand()
