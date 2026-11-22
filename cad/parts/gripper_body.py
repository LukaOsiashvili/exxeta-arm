"""Part 10: gripper body.

Role: turns on the wrist cap journal (J5 roll), driven by the J5 servo horn that
sits in a pocket of its base plate. It holds the J6 servo upright on the roll
axis and guides the two racks of the parallel gripper (ADR-0006).

How the gripper works: a pinion on the J6 servo meshes with two racks, one on
each side. Turning the servo pushes one rack left and the other right by the
same amount, so the two fingers open and close in parallel and the grip centre
stays on the hand axis.

Frame: wrist frame (origin on the J4 axis, +Z along the hand); fingers move along X.
Print: as modelled (skirt down) with support under the base plate.
"""
from common import EPS, HAND, M2_PILOT, PartDef, box, cyl_z, rbox_z, ring_z, trimmed_horn_sketch
from build123d import Plane, Polygon, Pos, Rot, extrude, mirror


def make():
    h, m = HAND, HAND.m
    gx0, gx1 = h.guide_x0, h.guide_x1
    half = h.half                                   # servo pocket half width incl. wall
    # --- base plate + skirt that rides on the journal ----------------------
    add = cyl_z(14.5, h.z_plate0, h.z_plate1) + rbox_z(gx0, gx1, -half, half, h.z_plate0, h.z_plate1, 2.0)
    add += ring_z(14.5, h.journal_r + 0.15, h.z_cap_top + 0.3, h.z_plate0 + EPS)
    cut = cyl_z(m.horn_hub_d / 2 + 0.2, h.z_plate0 - 1, h.z_plate1 + 1)           # J5 horn hub passes through
    cut += Pos(0, 0, h.z_recess) * extrude(trimmed_horn_sketch(), 5)              # trimmed J5 horn drops in from the top
    for deg in (0, 90, -90):                                                      # J5 alignment marks
        cut += Rot(0, 0, deg) * box(13.9, 15, -0.6, 0.6, h.z_cap_top, h.z_plate0 - 0.5)

    # --- J6 servo housing: the servo hangs by its ears, shaft up -------------
    add += rbox_z(gx0, gx1, -half, half, h.z_plate1 - EPS, h.rack_floor, 2.0)
    cut += box(h.bx0 - h.c, h.bx1 + h.c, -m.body_w / 2 - h.c, m.body_w / 2 + h.c, h.z_plate1 - EPS, h.rack_floor + 10)
    cut += box(h.X0 + 0.3, h.X1 - 0.3, -m.body_w / 2 - h.c, m.body_w / 2 + h.c, h.z6_ear, h.rack_floor + 10)  # room for the ears
    for dx in (-m.ear_hole_spacing / 2, m.ear_hole_spacing / 2):
        cut += cyl_z(M2_PILOT / 2, h.z6_ear - 9, h.z6_ear + 1, h.xc + dx, 0)
    cut += box(gx0 - 1, gx1 + 1, -3, 3, h.zb6, h.zb6 + 6)                         # cable exits, both ends

    # --- rack guides: a shelf and an outer wall on each side -----------------
    drop = h.wall_y1 - half
    # shelf on a 45 degree gusset (prints without support), then the guide wall
    prof = Plane.YZ * Polygon((half - EPS, h.rack_floor - drop), (h.wall_y1, h.rack_floor),
                              (h.wall_y1, h.rack_z1 + 0.1), (h.wall_y0, h.rack_z1 + 0.1),
                              (h.wall_y0, h.rack_floor), (half - EPS, h.rack_floor), align=None)
    wing = Pos(gx0, 0, 0) * extrude(prof, gx1 - gx0)
    # groove for the tongue on the back of the rack
    groove = box(gx0 - 1, gx1 + 1, h.wall_y0 - EPS, h.wall_y0 + 1.6, h.rack_z0 + 1.3, h.rack_z0 + 3.3)
    add += wing + mirror(wing, Plane.XZ)
    cut += groove + mirror(groove, Plane.XZ)
    return add - cut


PART = PartDef(10, "gripper_body", make, link="gripper", color="body",
               orientation="as modelled (skirt down)",
               supports="under the base plate around the skirt",
               notes="3 perimeters, 40 % infill. The rack grooves must be clean: remove any stringing.")
