"""Part 20: single-joint test rig, arm.

Role: the same fork as the upper arm and forearm, with a plain bar instead of a
link. Two holes sit exactly 50 mm and 100 mm from the joint axis: hang a known
weight there to measure the real torque of a servo
(torque in kgf*cm = weight in kg x distance in cm).

Frame: joint axis = Y through the origin, bar along +Z. Print: lying on the -X face.
"""
from common import EPS, JD, LARGE, PartDef, box, cyl_y, large_fork


def make():
    add, cut = large_fork(+1)
    z_a = LARGE.bridge_r + JD.bridge_t - EPS
    add += box(-8, 8, -5, 5, z_a, 110)
    for z in (50, 100):
        cut += cyl_y(2.0, -6, 6, 0, z)
    cut += box(7.2, 9, -6, 6, 99.5, 100.5) + box(7.2, 9, -6, 6, 49.5, 50.5)   # marks at 5 and 10 cm
    return add - cut


PART = PartDef(20, "test_rig_arm", make, link="none", color="accent", optional=True,
               orientation="lying on the -X face", supports="none",
               notes="Uses one 608 bearing, one pivot pin and one metal horn: the real parts, reusable in the arm.")
