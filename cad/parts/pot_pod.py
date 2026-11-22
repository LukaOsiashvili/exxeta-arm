"""Part 15: pot pod (optional, up to 3: J2, J3, J4). Only with pots.

Role: holds a B10K rotary pot (WH148 type, 6 mm shaft, M7 bushing) on the joint
axis, outside the bearing. The pot body is fixed to the fork (distal link); its
shaft is turned by the pivot pin (fixed to the servo housing) through the loose
pot coupler. The pot carries no load.

Frame: same as the pivot cover (bearing-arm outer face = y 0, pod on the -Y side).
Print: flange down.
"""
from common import PartDef, cyl_y

from parts.pivot_cover import flange

DEPTH = 19.0      # mounting face of the pot (outside of the end wall) below the fork arm


def make():
    pod = flange() + cyl_y(12.0, -DEPTH, 0)
    pod -= cyl_y(10.0, -DEPTH + 2.0, 1)                     # cavity for the coupler
    pod -= cyl_y(3.7, -DEPTH - 1, -DEPTH + 3)               # 7 mm pot bushing
    pod -= cyl_y(1.6, -DEPTH - 1, -DEPTH + 3, 7.8, 0)       # anti-rotation tab of the pot
    return pod


PART = PartDef(15, "pot_pod", make, qty=3, link="upper_arm", color="accent", optional=True,
               orientation="flange down", supports="none",
               notes="Pot body outside, nut inside. Replaces the pivot cover on joints with a pot.")
