"""Part 07: forearm (J3 -> J4).

Role: second moving link. A wide fork at the elbow (around the J3 servo housing),
a hollow spine that carries the cables, and a narrow fork at the wrist. The wrist
servo itself sits in the wrist block; this fork holds its horn on one side and
the 608 bearing on the other (ADR-0005).

Frame: origin on the J3 axis, +Z along the link, J4 axis at z = forearm_length.
Print: lying on the flat -X face.
"""
from common import (EPS, HW, JD, LARGE, MICRO, P, PartDef, bearing_arm, box, cyl_y, fork_marks, large_fork,
                    micro_horn_sketch, rbox_x, rbox_z, sketch_prism_xz)
from build123d import Pos


def make():
    L = P.geometry.forearm_length_mm
    s, mi, m = LARGE, MICRO, MICRO.m
    add, cut = large_fork(+1)

    # hollow spine between the two bridges (cables run inside)
    z_a = s.bridge_r + JD.bridge_t - EPS
    zw1 = L - mi.bridge_r                      # wrist bridge, face towards the wrist
    zw0 = zw1 - JD.bridge_t
    add += rbox_z(-HW, HW, -12, 12, z_a, zw0 + EPS, 3.0)
    t = JD.wall_thin
    cut += box(-HW + t, HW - t, -12 + t, 12 - t, s.bridge_r - 1, zw1 + 1)

    # wrist fork: bridge + two arms reaching forward to the J4 axis
    add += rbox_x(-HW, HW, -mi.W, mi.W, zw0, zw1, 1.5)
    add += Pos(0, 0, L) * (cyl_y(HW, mi.y_arm1_in, mi.W) + box(-HW, HW, mi.y_arm1_in, mi.W, zw0 - L, 0))
    # arm 1: pocket for the plastic servo horn (transmits the torque) + screw access
    pocket = sketch_prism_xz(micro_horn_sketch(), mi.y_arm1_in - EPS, mi.y_arm1_in + m.horn_blade_t + 0.1)
    cut += Pos(0, 0, L) * (pocket + cyl_y(2.6, mi.y_arm1_in - 1, mi.W + 1))
    # arm 2: 608 bearing seat
    a2, c2 = bearing_arm(mi.y_arm2_in, mi.y_arm2_out, zw0 - L, -17.0)
    add += Pos(0, 0, L) * a2
    cut += Pos(0, 0, L) * (c2 + fork_marks(mi.y_arm2_out - EPS, +1, 0))
    # zip-tie slots on the back of the spine
    for z in (z_a + 5, zw0 - 5):
        cut += box(-HW - 1, -HW + t + 1, -3.5, 3.5, z - 1.0, z + 1.0)
    return add - cut


PART = PartDef(7, "forearm", make, link="forearm", color="body",
               orientation="lying on the flat back (-X) face",
               supports="none",
               notes="The horn pocket must grip the MG90S double-arm horn: check with the tolerance test first.")
