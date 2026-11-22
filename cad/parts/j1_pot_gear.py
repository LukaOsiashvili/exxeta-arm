"""Part 17: gear for the optional J1 pot (only with pots).

Role: sits on the pot shaft and meshes 1:1 with the gear ring under the turntable,
so the pot turns exactly as far as the base yaw joint.

Frame: gear axis = Z, bottom face at z = 0. Print: flat.
"""
from common import P, PartDef, box, cyl_z, gear_sketch
from build123d import extrude


def make():
    t = P.turntable
    h = P.servo_mech.MG996R.horn_t
    gear = extrude(gear_sketch(t.pot_gear_module, t.pot_gear_teeth), h)
    c = P.tolerances.pot_socket_clearance_mm
    # D-shaped bore for a 6 mm D-shaft (flat 4.5 mm from the far side)
    bore = cyl_z(3.0 + c / 2, -1, h + 1) & box(-4, 1.5 + c / 2, -4, 4, -1, h + 1)
    return gear - bore


PART = PartDef(17, "j1_pot_gear", make, link="base", color="accent", optional=True,
               orientation="flat", supports="none", notes="0.1 mm layers if possible (1 mm module teeth).")
