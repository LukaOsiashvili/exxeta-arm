"""Part 13: pivot pin (3 needed: J2, J3, J4).

Role: goes through the 608 bearing from the outside into the socket on the servo
housing. An M3 x 16 screw through its centre clamps the inner ring of the bearing
to the housing and carries the load. The flats on the head drive the optional pot.

Frame: axis along Y; outer face of the head at y = 0, pin pointing towards +Y.
Print: head down.
"""
from common import BRG, JD, M3_CLEAR, PartDef, box, cyl_y


def make():
    d, t = JD.pivot_pin_flange_d, JD.pivot_pin_flange_t
    head = cyl_y(d / 2, 0, t) & box(-5.0, 5.0, -1, t + 1, -d, d)       # round head with two flats
    pin = cyl_y(BRG.id_mm / 2 - 0.03, t - 0.01, t + BRG.width_mm + 1.8)
    return head + pin - cyl_y(M3_CLEAR / 2, -1, 20)


PART = PartDef(13, "pivot_pin", make, qty=3, link="upper_arm", color="accent",
               orientation="head down", supports="none", notes="100 % infill.")
