"""Part 02: bottom panel of the base.

Role: removable service cover (screwed, not snapped). The battery holders and the
heavier modules are mounted on it in Phase 5, so the weight sits as low as possible.

Frame: world. Print: flat.
"""
from common import M3_CLEAR, M3_HEAD_D, P, PartDef, cyl_z
from build123d import RectangleRounded, extrude

from parts.base_shell import corner_posts


def make():
    b = P.base
    fx, fy = b.footprint_mm
    c = 0.3                                  # gap all round so the panel drops in easily
    plate = extrude(RectangleRounded(fx - 2 * b.wall_mm - 2 * c, fy - 2 * b.wall_mm - 2 * c,
                                     b.corner_radius_mm - b.wall_mm - c), b.bottom_panel_t_mm)
    for x, y in corner_posts():
        plate -= cyl_z(M3_CLEAR / 2, -1, b.bottom_panel_t_mm + 1, x, y)
        plate -= cyl_z(M3_HEAD_D / 2, -1, 1.6, x, y)             # screw heads sit below the surface
    for sx in (-1, 1):                                            # shallow seats for 4 stick-on rubber feet
        for sy in (-1, 1):
            plate -= cyl_z(6.5, -1, 0.5, sx * (fx / 2 - 30), sy * (fy / 2 - 14))
    return plate


PART = PartDef(2, "base_panel", make, link="base", color="accent",
               orientation="flat, outside face down", supports="none",
               notes="4 x M3 x 12 into the corner posts. Stick 4 rubber feet (12 mm) into the round seats.")
