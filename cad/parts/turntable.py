"""Part 05: turntable with shoulder tower.

Role: the rotating platform of J1 (base yaw). Its rim is the inner race of the
printed ball bearing, its hub screws onto the J1 servo horn, and the tower on top
holds the J2 (shoulder) servo.

Frame: origin on the J2 axis, +Z up (= J1 axis), +X forward, +Y = J2 axis (horn side).
Print: as modelled (disc on the bed, tower up). The V-groove is a 45 degree overhang
and prints without support; the servo pocket roof needs a short bridge.
"""
from common import (EPS, HW, JD, LARGE, M3_CLEAR, M3_HEAD_D, P, TT, PartDef, box, cyl_z, gear_sketch,
                    mg996r_cradle, polar, prism_xz, revolve_rz)
from build123d import Pos, Rot, extrude


def make():
    t = P.turntable
    sh = TT.z_shoulder                      # world height of this part's origin
    zbc, zwb, zwt, zrt = TT.z_bc - sh, TT.z_web_bot - sh, TT.z_web_top - sh, TT.z_rim_top - sh

    # --- disc: web + raised rim -------------------------------------------
    part = cyl_z(TT.r_disc, zwb, zrt) - cyl_z(t.rim_inner_radius, zwt, zrt + 1)
    # V-groove of the ball race, cut into the rim (two 45 degree faces)
    reach = TT.r_disc + 1 - (TT.rp - TT.a)
    part -= revolve_rz([(TT.rp - TT.a, zbc), (TT.r_disc + 1, zbc + reach), (TT.r_disc + 1, zbc - reach)])

    # --- hub: gear ring for the optional J1 pot, around the servo horn ----
    m = P.servo_mech.MG996R
    gear = extrude(gear_sketch(t.pot_gear_module, t.pot_gear_teeth), m.horn_t)
    part += Pos(0, 0, zwb - m.horn_t) * gear
    part -= cyl_z(m.horn_d / 2 + 0.3, zwb - m.horn_t - 1, zwb)       # the horn sits inside the gear ring
    part -= cyl_z(4.2, zwb - 1, zwt + 1)                              # horn centre screw access
    for k in range(4):                                                # 4 x M3 into the horn, heads sunk
        x, y = polar(m.horn_pcd / 2, 45 + 90 * k)
        part -= cyl_z(M3_CLEAR / 2, zwb - 1, zwt + 1, x, y)
        part -= cyl_z(M3_HEAD_D / 2, zwt - 3.0, zwt + 1, x, y)

    # --- cable hole behind the tower --------------------------------------
    part -= cyl_z(t.cable_hole_d / 2, zwb - 1, zwt + 1, -t.cable_hole_radius, 0)

    # --- J1 end-stop lug on top of the rim, at the back (meets the tabs on the clamp ring)
    part += box(-TT.r_disc + 0.3, -40.0, -3, 3, zrt - EPS, zrt + 4)

    # --- shoulder tower: MG996R cradle on two side walls -------------------
    add, cut = mg996r_cradle()
    s, w = LARGE, JD.wall
    z_box = s.z_near - 0.25 - w
    for x0, x1 in ((-HW, -HW + w), (HW - w, HW)):
        add += box(x0, x1, s.y_wall_out, s.y_ear, zwt - EPS, z_box + EPS)
    part = part + add - cut

    # --- J2 back stop = rest support --------------------------------------
    # The fork bridge of the upper arm lands on this block at about -75 degrees.
    # Parked (J2 = -72) and switched off, the arm sags 3 degrees onto it and stays there.
    part += prism_xz([(-HW + EPS, -18.7), (-HW - 10.7, -8.0), (-HW - 10.7, 4.0), (-HW + EPS, 4.0)], 0.0, LARGE.y_ear)

    # --- alignment marks on the rim top: 0 and +-90 degrees ----------------
    for deg in (0, 90, -90):
        part -= Rot(0, 0, deg) * box(TT.r_disc - 5, TT.r_disc + 1, -0.6, 0.6, zrt - 0.6, zrt + 1)
    return part


PART = PartDef(5, "turntable", make, link="turntable", color="body",
               orientation="as modelled: disc flat on the bed, tower pointing up",
               supports="none (45 degree groove, short bridges)",
               notes="Inner race of the J1 ball bearing: print slowly with 0.2 mm layers for a smooth groove.")
