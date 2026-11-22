"""Part 09: wrist cap (J5 journal).

Role: clamps the J5 servo by its ears and carries a round journal around the output
shaft. The gripper turns on this journal, so sideways loads from the gripper go
into the wrist block and not into the small servo shaft.

Frame: wrist frame (origin on the J4 axis, +Z along the hand).
Print: as modelled, flat frame on the bed.
"""
from common import EPS, HAND, PartDef, box, cyl_z, rbox_z, ring_z


def make():
    h, m = HAND, HAND.m
    part = rbox_z(h.X0, h.X1, -h.half, h.half, h.z_ear_top, h.z_cap_top, 2.0)
    # opening for the servo case below the top plate
    part -= box(h.bx0 - h.c, h.bx1 + h.c, -m.body_w / 2 - h.c, m.body_w / 2 + h.c, h.z_ear_top - 1, h.z_case)
    # journal ring
    part += ring_z(h.journal_r, m.boss_d / 2 + 0.4, h.z_cap_top - EPS, h.z_journal_top)
    # opening for the output boss and the small gear bump next to it
    part -= cyl_z(m.boss_d / 2 + 0.4, h.z_case - 1, h.z_journal_top + 1)
    part -= box(0, 9.0, -3.2, 3.2, h.z_case - 1, h.z_journal_top + 1)
    # ear screws go through the cap into the wrist block
    for dx in (-m.ear_hole_spacing / 2, m.ear_hole_spacing / 2):
        part -= cyl_z(1.2, h.z_ear_top - 1, h.z_cap_top + 1, h.xc + dx, 0)
        part -= cyl_z(2.4, h.z_ear_top + 1.5, h.z_cap_top + 1, h.xc + dx, 0)
    # index line for the J5 alignment marks
    part -= box(15.0, h.X1 + 1, -0.6, 0.6, h.z_cap_top - 0.6, h.z_cap_top + 1)
    return part


PART = PartDef(9, "wrist_cap", make, link="wrist", color="accent",
               orientation="as modelled (frame on the bed, journal up)",
               supports="a little, under the top plate inside the frame",
               notes="Fixed with two M2 x 10 self-tapping screws through the servo ears.")
