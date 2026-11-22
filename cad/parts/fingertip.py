"""Part 12: fingertip pad (2 needed, identical).

Role: swappable grip pad. A sleeve that slides over the end of a finger; the thick
wall is the grip face. Print in TPU for grip, or in PETG and stick rubber/tape on.

Frame: wrist frame, on the +X finger with the gripper closed (grip face at x = 0).
The pad for the other finger is the same part turned 180 degrees about the hand axis.
"""
from common import HAND, PartDef, box


def make():
    h = HAND
    t, c, length = 1.2, 0.15, 28.0
    outer = box(0, h.pad_t + h.finger_t + t, -h.fy - t, h.fy + t, h.z_tip - length, h.z_tip + t)
    cavity = box(h.pad_t - c, h.pad_t + h.finger_t + c, -h.fy - c, h.fy + c, h.z_tip - length - 1, h.z_tip + c)
    return outer - cavity


PART = PartDef(12, "fingertip", make, qty=2, link="finger_a", color="accent", material="TPU (or PETG + rubber tape)",
               orientation="open end up", supports="none",
               notes="Push fit. If loose, add a drop of glue or one wrap of tape on the finger.")
