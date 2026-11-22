"""Part 04: clamp ring (upper half of the J1 ball race).

Role: screwed onto the base deck after the turntable and the steel balls are in
place. Its 45 degree face closes the ball race from above, so the turntable can
take weight, side load and tipping moment without loading the J1 servo.
Two tabs on top are the J1 end stops; a notch marks the J1 zero position.

Frame: world. Print: upside down (flat top on the bed).
"""
from common import EPS, M3_CLEAR, M3_HEAD_D, P, TT, PartDef, box, cyl_z, polar, revolve_rz
from build123d import Rot

from parts.base_shell import CLAMP_SCREW_DEG, CLAMP_SCREW_R, RING_OUT

RING_H = 6.0
STOP_DEG = 99.5     # the lug on the turntable touches a tab at about +-91.5 degrees of yaw


def make():
    H = P.base.height_mm
    reach = TT.rp + TT.a - TT.r_ring
    ring = revolve_rz([(TT.r_ring, H + reach), (TT.rp + TT.a, H), (RING_OUT, H), (RING_OUT, H + RING_H),
                       (TT.r_ring, H + RING_H)])
    for deg in CLAMP_SCREW_DEG:
        x, y = polar(CLAMP_SCREW_R, deg)
        ring -= cyl_z(M3_CLEAR / 2, H - 1, H + RING_H + 1, x, y)
        ring -= cyl_z(M3_HEAD_D / 2, H + RING_H - 3.2, H + RING_H + 1, x, y)
    # end-stop tabs reach in over the turntable rim, at the back
    zt = H + RING_H
    for deg in (180 - STOP_DEG, 180 + STOP_DEG):
        tab = box(40.5, TT.r_ring + EPS, -3, 3, zt + 0.5, zt + 4) + box(TT.r_ring, TT.r_ring + 4, -3, 3, zt - EPS, zt + 4)
        ring += Rot(0, 0, deg) * tab
    # index notch for the J1 alignment marks (front)
    ring -= box(TT.r_ring - 1, TT.r_ring + 3.5, -0.6, 0.6, zt - 0.6, zt + 1)
    return ring


PART = PartDef(4, "clamp_ring", make, link="base", color="accent",
               orientation="upside down (flat top face on the bed)", supports="none",
               notes="6 x M3 x 12. Tighten evenly until the turntable turns smoothly without rocking; "
                     "add paper shims under the ring if it binds.")
