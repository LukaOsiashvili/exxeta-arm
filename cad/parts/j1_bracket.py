"""Part 03: J1 servo bracket.

Role: holds the base-yaw servo (MG996R, shaft up) under the turntable, and the
optional J1 pot beside it. Screwed from below to four bosses inside the base.
The servo only turns the turntable; the ball race carries the arm.

The bracket covers only the front of the opening: the cables from the arm hang
down through the back and need that space to swing with the turntable.

Frame: world. Print: flat, top face up.
"""
import math

from common import M3_CLEAR, M3_NUT_AF, P, TT, PartDef, box, cyl_z, hex_prism_z, polar
from build123d import Polygon, Pos, extrude

from parts.base_shell import BRACKET_BOSS_DEG, BRACKET_SCREW_R


def make():
    m, t = P.servo_mech.MG996R, P.turntable
    z1 = TT.z_bracket_top
    z0 = z1 - TT.bracket_t
    # front sector (+-62 degrees) plus a strip that reaches back under the rear servo ear
    pts = [(0.0, 0.0)] + [polar(47.5, d) for d in range(-62, 63, 4)]
    plate = Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), TT.bracket_t) + box(-22, 20, -14.5, 14.5, z0, z1)
    # servo: body towards the front (+X), shaft on the J1 axis
    c = P.tolerances.servo_pocket_clearance_mm
    bx0, bx1 = -m.shaft_offset, m.body_l - m.shaft_offset
    plate -= box(bx0 - c, bx1 + c, -m.body_w / 2 - c, m.body_w / 2 + c, z0 - 1, z1 + 1)
    xc = (bx0 + bx1) / 2
    for dx in (-m.ear_hole_spacing_l / 2, m.ear_hole_spacing_l / 2):
        for dy in (-m.ear_hole_spacing_w / 2, m.ear_hole_spacing_w / 2):
            plate -= cyl_z(M3_CLEAR / 2, z0 - 1, z1 + 1, xc + dx, dy)
            plate -= hex_prism_z(M3_NUT_AF, z0 - 1, z0 + 2.2, xc + dx, dy)      # nuts held from below
    # mounting screws into the base bosses
    for deg in BRACKET_BOSS_DEG:
        x, y = polar(BRACKET_SCREW_R, deg)
        plate -= cyl_z(M3_CLEAR / 2, z0 - 1, z1 + 1, x, y)
    # optional J1 pot: 7 mm bushing hole + hole for its anti-rotation tab
    px, py = polar(TT.gear_pd, t.pot_angle_deg)
    plate -= cyl_z(3.7, z0 - 1, z1 + 1, px, py)
    ax, ay = polar(7.8, t.pot_angle_deg + 90)
    plate -= cyl_z(1.6, z0 - 1, z1 + 1, px + ax, py + ay)
    return plate


PART = PartDef(3, "j1_bracket", make, link="base", color="accent",
               orientation="flat, top face up", supports="none",
               notes="5 perimeters or 60 % infill: it takes the J1 servo torque.")
