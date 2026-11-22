"""Part 01: base shell.

Role: the heavy, closed base. Its deck carries the lower half of the J1 ball race;
a skirt below the deck holds the J1 servo bracket. All electronics, the battery
and the wiring live inside (mounting details for the modules are added in Phase 5).

Frame: world (table = z 0, J1 axis = Z, +X = front).
Print: upside down (deck on the bed). No supports: everything grows from the deck.
"""
from common import EPS, M3_CLEAR, M3_NUT_AF, M3_NUT_T, P, TT, PartDef, box, cyl_x, cyl_z, polar, prism_xz, revolve_rz
from build123d import Axis, Plane, Pos, RectangleRounded, Rot, Text, chamfer, extrude

DECK_T = 3.0
RING_OUT = 56.0          # outer radius of the race ring under the deck
RING_T = 6.0
SKIRT_T = 4.0
CLAMP_SCREW_R = 52.5
CLAMP_SCREW_DEG = [30 + 60 * k for k in range(6)]
BRACKET_BOSS_DEG = [25, -25, 55, -55]
BRACKET_SCREW_R = 42.5
POST_INSET = 9.0         # corner posts for the bottom panel
REAR_OPENING = (100.0, 34.0)   # width (Y) x height (Z) of the opening for the rear panel
REAR_OPENING_Z0 = 12.0


def corner_posts():
    b = P.base
    fx, fy = b.footprint_mm
    return [(sx * (fx / 2 - b.wall_mm - POST_INSET), sy * (fy / 2 - b.wall_mm - POST_INSET))
            for sx in (-1, 1) for sy in (-1, 1)]


def make():
    b = P.base
    fx, fy = b.footprint_mm
    H, w, r = b.height_mm, b.wall_mm, b.corner_radius_mm
    shell = extrude(RectangleRounded(fx, fy, r), H)
    shell = chamfer(shell.edges().group_by(Axis.Z)[-1], 2.0)          # soft top edge
    groove = extrude(RectangleRounded(fx + 2, fy + 2, r + 1), 1.6) - extrude(RectangleRounded(fx - 1.6, fy - 1.6, r - 0.8), 1.6)
    shell -= Pos(0, 0, 7.0) * groove                                  # shadow line above the bottom panel
    shell -= Pos(0, 0, -1) * extrude(RectangleRounded(fx - 2 * w, fy - 2 * w, r - w), H - DECK_T + 1)

    # --- J1 race ring + skirt, hanging under the deck -----------------------
    zbt = TT.z_bracket_top
    add = cyl_z(RING_OUT, H - RING_T, H) + cyl_z(TT.r_ring + SKIRT_T, zbt, H)
    cut = cyl_z(TT.r_ring, 0, H + 1)
    # lower V-face of the ball race (45 degrees)
    cut += revolve_rz([(TT.r_ring - 1, H - (TT.rp + TT.a - TT.r_ring) - 1), (TT.rp + TT.a, H),
                       (TT.rp + TT.a, H + 1), (TT.r_ring - 1, H + 1)])
    # clamp-ring screws: M3 from the top into nuts pushed in from the side, under the deck
    for deg in CLAMP_SCREW_DEG:
        x, y = polar(CLAMP_SCREW_R, deg)
        cut += cyl_z(M3_CLEAR / 2, H - RING_T - 1, H + 1, x, y)
        cut += Rot(0, 0, deg) * box(CLAMP_SCREW_R - M3_NUT_AF / 2 - 0.3, RING_OUT + 1, -M3_NUT_AF / 2, M3_NUT_AF / 2,
                                    H - DECK_T - M3_NUT_T, H - DECK_T)
    # bosses for the J1 servo bracket (it is screwed on from below)
    for deg in BRACKET_BOSS_DEG:
        boss = prism_xz([(38.5, zbt), (TT.r_ring + 1, zbt), (TT.r_ring + 1, zbt + 16.5), (38.5, zbt + 7)], -7, 7)
        add += Rot(0, 0, deg) * boss
        x, y = polar(BRACKET_SCREW_R, deg)
        cut += cyl_z(M3_CLEAR / 2, zbt - 1, zbt + 6, x, y)
        cut += Rot(0, 0, deg) * box(38.0, BRACKET_SCREW_R + M3_NUT_AF / 2 + 0.3, -M3_NUT_AF / 2, M3_NUT_AF / 2,
                                    zbt + 2.5, zbt + 2.5 + M3_NUT_T)
    shell = shell + add - cut

    # --- corner posts for the bottom panel (M3 from below into side-loaded nuts)
    for x, y in corner_posts():
        shell += cyl_z(6.0, b.bottom_panel_t_mm, 22, x, y)
        shell -= cyl_z(M3_CLEAR / 2, b.bottom_panel_t_mm - 1, 16, x, y)
        sx = 1 if x > 0 else -1
        shell -= box(x - sx * 7, x + sx * 3.2, y - M3_NUT_AF / 2, y + M3_NUT_AF / 2, 9.0, 9.0 + M3_NUT_T)

    # --- openings -----------------------------------------------------------
    ow, oh = REAR_OPENING
    shell -= box(-fx / 2 - 1, -fx / 2 + w + 1, -ow / 2, ow / 2, REAR_OPENING_Z0, REAR_OPENING_Z0 + oh)   # rear panel
    for sy in (-1, 1):                                             # rear-panel screws (self-tapped M3)
        shell -= cyl_x(1.3, -fx / 2 - 1, -fx / 2 + w + 1, sy * (ow / 2 + 5), REAR_OPENING_Z0 + oh / 2)
    for sy in (-1, 1):                                             # ventilation slots in both side walls
        for k in range(6):
            x0 = -30 + k * 8
            shell -= box(x0, x0 + 3, sy * (fy / 2 - w - 1), sy * (fy / 2 + 1), 14, 38)
    shell -= cyl_x(2.6, fx / 2 - w - 1, fx / 2 + 1, -55, 14)       # status LED light pipe (5 mm)
    if P.options.emboss_exxeta:
        txt = Plane(origin=(fx / 2, 0, 34), x_dir=(0, 1, 0), z_dir=(1, 0, 0)) * Text("EXXETA", 17)
        shell -= extrude(txt, -0.8)
    return shell


PART = PartDef(1, "base_shell", make, link="base", color="body",
               orientation="upside down (deck on the bed)",
               supports="none",
               notes="Largest part (170 x 170 x 65). 3 perimeters, 15 % infill. The lower ball-race face is on the deck: "
                     "use a clean, level bed.")
