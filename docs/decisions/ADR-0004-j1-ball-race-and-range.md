# ADR-0004: J1 turntable bearing and yaw range

- **Status:** accepted (implementer choice, Phase 1)
- **Spec:** §4.4 (base yaw needs proper thrust and radial support), §4.1 (range set by the cable loop)

## Decision

**Printed four-point ball race with 46 loose 6 mm steel balls**, pitch diameter 92 mm.

- The rim of the turntable has a V-groove (inner race). The base deck carries the lower 45° face of the outer race; a screwed-on **clamp ring** carries the upper 45° face.
- Each ball touches four faces, so the one bearing takes weight, side load and tipping moment. The J1 servo only turns the table.
- Preload is set with the six clamp-ring screws (paper shims if it binds).

**J1 range: ±90°**, with hard stops at about ±91.5° (a lug on the turntable rim meets two tabs on the clamp ring).

## Why

- A lazy-susan bearing is sloppy and hard to find in a fitting size; a large thin-section bearing is expensive and not a common part here. Steel balls are sold by bicycle and bearing shops everywhere. `turntable.ball_d` can be set to 6.35 mm (1/4 inch bicycle balls).
- The V-faces are 45° overhangs, so all three parts print without support.
- A "608 in the centre" (the spec's other suggestion) is not possible: the servo horn sits on the axis.

## Why ±90° and not more

The servo horn occupies the centre of the turntable, so the cables cannot go through the exact centre as §4.4 assumed. They pass through an 11 mm hole 30 mm behind the axis and hang into the base. That hole sweeps an arc as the table turns, and the gear of the optional J1 pot sits beside the servo at the front. At more than ±90° the cable bundle would run into that gear. ±90° is inside the range the spec suggests (±90° to ±120°).

## Consequences

- BOM: 46 + spares steel balls, 6 mm.
- The race needs a clean print (0.2 mm layers, slow outer walls) and a little grease.
- Listed in `unverified.md`: smoothness and play of the printed race can only be judged on a real print.
