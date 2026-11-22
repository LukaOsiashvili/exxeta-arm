"""Part 08: wrist block.

Role: holds two MG90S servos. The J4 (wrist pitch) servo lies across the block with
its shaft on the J4 axis; the J5 (wrist roll) servo stands in front of it with its
shaft pointing along the hand.

Frame: origin on the J4 axis, +Y = J4 axis (horn side), +Z along the hand (= J5 axis).
Print: standing on its back (-Z) face.
"""
from common import EPS, HAND, HW, M2_PILOT, MICRO, PartDef, box, cyl_y, cyl_z, pivot_seat_cut, rbox_y, rbox_z


def make():
    h, mi, m = HAND, MICRO, HAND.m
    # --- J4 servo housing --------------------------------------------------
    add = rbox_y(h.X0, h.X1, mi.y_wall_out, mi.y_ear, -h.half, h.half, 2.0)
    add += cyl_y(6.0, mi.y_boss, mi.y_wall_out)                       # pivot boss for the 608 bearing
    cut = box(h.bx0 - h.c, h.bx1 + h.c, mi.y_wall_in, mi.y_ear + 30, -m.body_w / 2 - h.c, m.body_w / 2 + h.c)
    for dx in (-m.ear_hole_spacing / 2, m.ear_hole_spacing / 2):      # the servo comes with self-tapping screws
        cut += cyl_y(M2_PILOT / 2, mi.y_ear - 9, mi.y_ear + 1, h.xc + dx, 0)
    cut += box(h.X0 - 1, h.X1 + 1, mi.y_wall_in, mi.y_wall_in + 6, -3, 3)   # cable exits
    cut += pivot_seat_cut(mi.y_wall_out, mi.y_boss, mi.y_wall_in)

    # --- J5 servo housing (in front, shaft along +Z) -----------------------
    add += rbox_z(h.X0, h.X1, -h.half, h.half, h.z0 - 2.0, h.z_ear, 2.0)
    cut += box(h.bx0 - h.c, h.bx1 + h.c, -m.body_w / 2 - h.c, m.body_w / 2 + h.c, h.z0, h.z_ear + 30)
    for dx in (-m.ear_hole_spacing / 2, m.ear_hole_spacing / 2):
        cut += cyl_z(M2_PILOT / 2, h.z_ear - 9, h.z_ear + 1, h.xc + dx, 0)
    cut += box(h.X0 - 1, h.X1 + 1, -3, 3, h.z0, h.z0 + 6)
    # index line for the J4 alignment marks (on the -Y face, pointing along the hand)
    cut += box(-0.6, 0.6, -h.half - EPS, -h.half + 0.6, HW + 0.8, h.z_ear - 1)
    return add - cut


PART = PartDef(8, "wrist_block", make, link="wrist", color="body",
               orientation="standing on the -Z face (J5 pocket opening up)",
               supports="none",
               notes="Thin walls: 3 perimeters, 100 % infill is fine (tiny part).")
