"""Part 18: tolerance test plate. PRINT THIS FIRST.

Role: every printer (and print service) makes holes a little differently. This
small plate has each critical fit three times, at three clearances. Try your real
bearing, nut, servo horn and pot in it, pick the fit that feels right, and put
that value into `tolerances` in arm_params.json before printing the real parts.

Rows, left to right = tighter, default, looser:
  row 1: 608 bearing seat (should slide in by hand, without rattle)
  row 2: M3 screw hole | M3 nut pocket | pot D-shaft socket
  row 3: MG90S horn pocket (the horn must drop in and not rock)
  right: MG90S body pocket (one size, tests servo_pocket_clearance)

Print: flat, as modelled.
"""
from common import BRG, P, TOL, PartDef, box, cyl_z, hex_prism_z, micro_horn_sketch
from build123d import Pos, Rot, extrude

T = 9.0            # same thickness as a fork bearing arm
STEPS = (-0.1, 0.0, 0.1)   # added to the default clearance


def make():
    m = P.servo_mech.MG90S
    plate = box(0, 122, 0, 62, 0, T)
    for i, d in enumerate(STEPS):
        x = 16 + i * 29
        # row 1: bearing seats (through hole with a small lip, like the real arm)
        plate -= cyl_z(BRG.od_mm / 2 + TOL.bearing_seat_clearance_mm + d / 2, 0.6, T + 1, x, 46)
        plate -= cyl_z(BRG.od_mm / 2 - 1.3, -1, T + 1, x, 46)
        # row 2: screw hole, nut pocket, pot socket
        xs = 8 + i * 29
        plate -= cyl_z(1.5 + TOL.hole_clearance_mm + d / 2, -1, T + 1, xs, 24)
        plate -= hex_prism_z(5.5 + 2 * TOL.nut_pocket_clearance_mm + d, T - 2.6, T + 1, xs + 9, 24)
        plate -= cyl_z(1.7, -1, T + 1, xs + 9, 24)
        c = TOL.pot_socket_clearance_mm + d
        plate -= Pos(xs + 19, 24, 0) * (cyl_z(3.0 + c / 2, T - 8, T + 1) & box(-4, 1.5 + c / 2, -4, 4, T - 8, T + 1))
        # row 3: horn pockets
        horn = extrude(micro_horn_sketch(0.2 + d), 5)
        plate -= Pos(20 + i * 38, 8, T - m.horn_blade_t) * Rot(0, 0, 90) * horn
        # tick marks: 1, 2, 3 notches on the front edge tell the columns apart
        for k in range(i + 1):
            plate -= box(x - 3 + k * 3, x - 2 + k * 3, 60.5, 63, T - 0.8, T + 1)
    # MG90S body pocket
    c = TOL.servo_pocket_clearance_mm
    plate -= box(96, 96 + m.body_l + 2 * c, 20, 20 + m.body_w + 2 * c, -1, T + 1)
    return plate


PART = PartDef(18, "tolerance_test", make, link="none", color="body", optional=True,
               orientation="flat", supports="none",
               notes="Print with exactly the material and settings planned for the arm.")
