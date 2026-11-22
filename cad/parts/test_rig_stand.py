"""Part 19: single-joint test rig, stand.

Role: one MG996R housing on a foot, identical to the elbow and shoulder housings.
Together with the test arm (part 20) it is a complete pitch joint on the bench:
it checks that the servo, horn, bearing and pivot pin fit BEFORE the big parts
are printed, and it is the rig for measuring real servo torque (hang a weight on
the arm at 10 cm) and for the first firmware tests.

Frame: joint axis = Y through the origin, foot below. Print: lying on the -X face.
"""
from common import EPS, HW, JD, LARGE, PartDef, box, cyl_z, mg996r_cradle

FOOT_Z = -62.0


def make():
    s, w = LARGE, JD.wall
    add, cut = mg996r_cradle()
    z_box = s.z_near - 0.25 - w
    for x0, x1 in ((-HW, -HW + w), (HW - w, HW)):               # two legs, like the shoulder tower
        add += box(x0, x1, s.y_wall_out, s.y_ear, FOOT_Z, z_box + EPS)
    foot = box(-HW, 60, -45, 45, FOOT_Z, FOOT_Z + 5)             # flat on the bench, clamp or screw it down
    for x, y in ((50, 35), (50, -35), (0, 37), (0, -37)):
        foot -= cyl_z(2.2, FOOT_Z - 1, FOOT_Z + 6, x, y)
    return add + foot - cut


PART = PartDef(19, "test_rig_stand", make, link="none", color="body", optional=True,
               orientation="lying on the -X face (same as the arm links)", supports="none",
               notes="Clamp it to the table edge before hanging weights on the arm.")
