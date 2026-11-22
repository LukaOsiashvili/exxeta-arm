"""Part 16: pot coupler (optional, up to 3). Only with pots.

Role: connects the flat-sided head of the pivot pin to the D-shaft of the pot.
Both ends are a slightly loose fit, so small misalignment between pot and joint
axis does no harm and the pot never carries load (PROJECT_SPEC 4.5).

Frame: same as the pivot cover (y 0 = outer face of the fork's bearing arm).
Print: standing, pot end up.
"""
from common import JD, P, PartDef, box, cyl_y

Y_HEAD = 0.9       # the coupler reaches this far into the fork arm, over the pin head
Y_SCREW = -4.2     # end of the cavity for the M3 screw head
Y_END = -12.0      # pot-side end


def make():
    c = P.tolerances.pot_socket_clearance_mm
    d = JD.pivot_pin_flange_d
    body = cyl_y(7.0, Y_END, Y_HEAD)
    # socket for the pin head (round with two flats)
    head = cyl_y(d / 2 + c, -0.9, Y_HEAD + 1) & box(-5.0 - c, 5.0 + c, -1, Y_HEAD + 1, -d, d)
    screw = cyl_y(3.4, Y_SCREW, -0.8)
    # D-socket for a 6 mm D-shaft (flat at 4.5 mm)
    shaft = cyl_y(3.0 + c / 2, Y_END - 1, Y_SCREW + 0.1) & box(-4, 1.5 + c / 2, Y_END - 1, Y_SCREW + 0.1, -4, 4)
    return body - head - screw - shaft


PART = PartDef(16, "pot_coupler", make, qty=3, link="upper_arm", color="accent", optional=True,
               orientation="standing, pot end up", supports="none",
               notes="For knurled (split) pot shafts set options.pot_shaft = knurled6 and drill the socket to 6 mm.")
