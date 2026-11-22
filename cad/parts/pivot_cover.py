"""Part 14: pivot cover (3 needed: J2, J3, J4). Used when NO pot is fitted.

Role: closes the bearing side of a pitch joint cleanly and keeps dust out of the
608 bearing. With a pot fitted, the pot pod (part 15) takes its place.

Frame: outer face of the fork's bearing arm = plane y = 0, joint axis = Y through
the origin, cover on the -Y side; its two screws sit at z = +17 (towards the link).
Print: flat face down.
"""
from common import HW, M3_CLEAR, PartDef, box, cyl_y

SCREW_Z, SCREW_X = 17.0, 7.0
FLANGE_T = 2.0


def flange():
    """Flat plate shared by the cover and the pot pod, with the two screw holes."""
    f = cyl_y(HW - 1.0, -FLANGE_T, 0) + box(-HW + 1.0, HW - 1.0, -FLANGE_T, 0, 0, SCREW_Z + 5)
    for dx in (-SCREW_X, SCREW_X):
        f -= cyl_y(M3_CLEAR / 2, -FLANGE_T - 1, 1, dx, SCREW_Z)
    return f


def make():
    dome = cyl_y(9.0, -6.5, 0) - cyl_y(7.3, -5.0, 1)      # room for the pin head and its screw
    return flange() - cyl_y(7.3, -5.0, 1) + dome


PART = PartDef(14, "pivot_cover", make, qty=3, link="upper_arm", color="accent",
               orientation="flat face down", supports="none", notes="2 x M3 x 12 each, into the nuts in the fork arm.")
