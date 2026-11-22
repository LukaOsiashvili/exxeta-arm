"""Part 06: upper arm (J2 -> J3).

Role: the first moving link. Its fork straddles the shoulder tower (J2); its far
end is the housing of the elbow servo (J3).

Frame: origin on the J2 axis, +Z along the link, J3 axis at z = upper_arm_length.
Print: lying on the flat -X face.
"""
from common import EPS, HW, JD, LARGE, P, PartDef, box, large_fork, mg996r_cradle
from build123d import Pos


def make():
    L = P.geometry.upper_arm_length_mm
    s, w = LARGE, JD.wall
    add, cut = large_fork(+1)
    c_add, c_cut = mg996r_cradle()
    add += Pos(0, 0, L) * c_add
    cut += Pos(0, 0, L) * c_cut
    # two side rails join the fork bridge to the servo housing; the gap between them is the cable route
    z_a = s.bridge_r + JD.bridge_t - EPS
    z_b = L + s.z_near - 0.25 - w + EPS
    for x0, x1 in ((-HW, -HW + w), (HW - w, HW)):
        add += box(x0, x1, s.y_wall_out, s.y_ear, z_a, z_b)
    # cable passage through the bridge, and zip-tie slots in the back rail
    cut += box(-7, 7, s.y_wall_out + 3, -2.0, s.bridge_r - 1, s.bridge_r + JD.bridge_t + 1)
    for z in (z_a + 5, z_b - 5):
        cut += box(-HW - 1, -HW + w + 1, -3.5, 3.5, z - 1.0, z + 1.0)
    return add - cut


PART = PartDef(6, "upper_arm", make, link="upper_arm", color="body",
               orientation="lying on the flat back (-X) face",
               supports="none; the servo pocket roof is a 41 mm bridge",
               notes="4 perimeters. The 608 seat must be a sliding fit: check with the tolerance test first.")
