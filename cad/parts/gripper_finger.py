"""Part 11: gripper finger with rack (2 needed, identical).

Role: one sliding finger of the parallel gripper. The toothed bar (rack) meshes
with the pinion on the J6 servo; a tongue on its back runs in the groove of the
gripper body so the finger cannot lift out. The second finger is the same part
turned 180 degrees about the hand axis, on the other side of the pinion.

Frame: wrist frame, gripper CLOSED, this is the finger on the +Y rack (it moves
towards +X when the gripper opens).
Print: lying on the flat outer (+X) end face, or on its back with supports.
"""
from common import EPS, HAND, PartDef, box, rack_sketch
from build123d import Pos, extrude


def make():
    h = HAND
    rack = Pos(0, 0, h.rack_z0) * extrude(
        rack_sketch(h.pin_mod, h.rack_x0, h.rack_x1, h.rack_tip_y, h.rack_root_y, h.rack_back_y), h.rack_z1 - h.rack_z0)
    tongue = box(h.rack_x0, h.rack_x1, h.rack_back_y - EPS, h.rack_back_y + 1.4, h.rack_z0 + 1.5, h.rack_z0 + 3.1)
    fx0, fx1 = h.pad_t, h.pad_t + h.finger_t
    # riser from the back of the rack up to the finger root (stays clear of the pinion)
    riser = box(fx0 - 5, fx1, h.rack_root_y, h.rack_back_y, h.rack_z1 - EPS, h.z_finger0 + 3)
    root = box(fx0, fx1, -h.fy, h.rack_back_y, h.z_finger0, h.z_finger0 + 3)
    finger = box(fx0, fx1, -h.fy, h.fy, h.z_finger0, h.z_tip)
    return rack + tongue + riser + root + finger


PART = PartDef(11, "gripper_finger", make, qty=2, link="finger_a", color="accent",
               orientation="lying on its back (rack teeth pointing up)",
               supports="a little under the finger",
               notes="100 % infill. Lightly grease the rack back and tongue.")
