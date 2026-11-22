"""Part 21: gripper pinion.

Role: the small gear on the J6 servo. It sits on the round boss of the servo; the
servo horn (cut to 12 mm) drops into the pocket on top and its screw clamps
everything, so the horn drives the pinion and the pinion drives both racks.

Frame: gear axis = Z through the origin, in the wrist frame (z = real position).
Print: flat, pocket up.
"""
from common import HAND, PartDef, cyl_z, gear_sketch, trimmed_horn_sketch
from build123d import Pos, Rot, extrude


def make():
    h, m = HAND, HAND.m
    gear = Pos(0, 0, h.pin_z0) * extrude(gear_sketch(h.pin_mod, h.pin_teeth), h.pin_z1 - h.pin_z0)
    gear -= cyl_z(m.horn_hub_d / 2 + 0.2, h.pin_z0 - 1, h.pin_z1 + 1)
    gear -= Pos(0, 0, h.pin_z1 - m.horn_blade_t - 0.1) * extrude(trimmed_horn_sketch(), 5)
    return gear


PART = PartDef(21, "gripper_pinion", make, link="pinion", color="accent",
               orientation="flat, horn pocket up", supports="none",
               notes="0.1 mm layers if possible. 100 % infill.")
