# Kinematics explained

Code: `dora/exxeta_arm/kinematics.py`. Tests: `tests/test_core.py` (3000 random poses solved back to within a nanometre).

## Frames and angles

URDF-style frames, the same as in the CAD (`cad/assembly.py`) and the generated `description/exxeta_arm.urdf`.

- World: origin on the table under the base axis. **X forward, Y left, Z up.**
- **Zero pose: the arm points straight up.**

| Joint | Axis | 0 means | Positive direction |
|---|---|---|---|
| J1 base yaw | Z | arm faces forward (+X) | counter-clockwise seen from above |
| J2 shoulder | Y | upper arm vertical | leaning forward |
| J3 elbow | Y | forearm in line with upper arm | bending forward |
| J4 wrist pitch | Y | hand in line with forearm | bending forward |
| J5 wrist roll | hand axis | fingers move sideways | right-hand rule along the hand |
| J6 gripper | (servo) | closed | opening |

Lengths (from `arm_params.json`): shoulder height **H = 106.5 mm**, upper arm **L1 = 80**, forearm **L2 = 80**, wrist axis to grip centre **L3 = 95**.

## The key idea: look at the arm from the side

J1 turns the whole arm about the vertical axis. J2, J3 and J4 all turn about parallel axes. So if you stand beside the arm and look along those axes, you see a **flat three-link arm**:

```
        z ^
          |            a2 = q2+q3
          |        elbow o--------o wrist
          |             /  L2      \  L3      a3 = q2+q3+q4
          |        L1  /            \
          |           /  a1 = q2     o grip centre (r, z)
          |  shoulder o
          |     H     |
          +-----------+--------------------> r   (distance from the base axis)
```

`a1, a2, a3` are the directions of the three links **measured from straight up**. Because the joint axes are parallel, the angles simply add: `a1 = q2`, `a2 = q2 + q3`, `a3 = q2 + q3 + q4`.

The hand's direction `a3` is what the software calls **pitch**: 0° = pointing up, 90° = horizontal, 180° = pointing straight down.

## Forward kinematics (angles → position)

Walk along the links and add up:

```
r = L1·sin(a1) + L2·sin(a2) + L3·sin(a3)
z = H + L1·cos(a1) + L2·cos(a2) + L3·cos(a3)

x = r·cos(q1)        y = r·sin(q1)
pitch = a3           roll = q5
```

Check with the zero pose: all angles 0, so r = 0 and z = H + L1 + L2 + L3 = 361.5 mm. Straight up. And with q2 = 90°: r = 255 mm, z = H. Fully stretched forward, which is the reach used in the torque budget.

## Inverse kinematics (position → angles)

The target is five numbers: x, y, z, pitch, roll. The arm has five joints, so there is generally a solution (two, in fact: elbow up and elbow down).

**Step 1: base yaw.** Seen from above, the arm must point at the target:

```
q1 = atan2(y, x)          r = sqrt(x² + y²)
```

**Step 2: find the wrist.** We know where the grip centre should be and which way the hand points, so we know where the wrist must be: one hand-length back along the hand.

```
rw = r − L3·sin(pitch)
zw = z − H − L3·cos(pitch)
```

**Step 3: two-link triangle.** Shoulder, elbow and wrist form a triangle with sides L1, L2 and D = √(rw² + zw²). The law of cosines gives the elbow angle:

```
cos(q3) = (D² − L1² − L2²) / (2·L1·L2)
```

- If this is greater than 1 the target is **too far**; if it is less than −1 it is **too close**. The code reports that; it never clamps silently.
- Otherwise `q3 = ± acos(…)`: the two signs are the two elbow solutions.

**Step 4: shoulder.** The direction from shoulder to wrist, minus the angle the bent elbow takes up:

```
q2 = atan2(rw, zw) − atan2(L2·sin(q3), L1 + L2·cos(q3))
```

**Step 5: wrist.** The three angles must add up to the pitch we asked for:

```
q4 = pitch − q2 − q3          q5 = roll
```

**Step 6: choose.** Throw away solutions outside the joint limits. Of those left, take the one closest to the arm's current pose, so the arm does not flip its elbow for no reason.

## When the exact pose is impossible

Often a point is reachable, but not with the hand pointing the way you asked (for example pointing straight down at full stretch). With `allow_pitch_relax` the solver tries nearby pitch angles in 2.5° steps, up to ±90°, and returns the first that works, flagged `pitch_relaxed`. The web app uses this while you drag the 3D target and tells you "reachable (hand angle will change)".

## Limits of this model

- A target **behind** the base (`atan2` outside ±90°) is reported as out of J1's range. The arm could reach some of those points by leaning backwards over its own base; the solver does not try that.
- Directly above the base axis (r ≈ 0) the yaw is undefined; the solver keeps the current yaw.
- The gripper's opening is not part of IK. The grip centre does not move when the gripper opens, because both fingers move symmetrically.
- The model is rigid. A real arm with hobby servos sags a few millimetres under load.

## Straight lines

A joint move swings the hand on an arc. For a straight line the planner (`planner.py`) cuts the line into 3 mm pieces, solves IK for every piece starting from the previous solution, and refuses the whole move up front if any piece is unreachable or would need a joint to jump by more than 20°.
