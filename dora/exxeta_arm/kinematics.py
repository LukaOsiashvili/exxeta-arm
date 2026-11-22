"""Forward and inverse kinematics of exxeta-arm (5 joints + gripper).

Role in the system
------------------
Pure maths, no dora: used by the `kinematics` node, the planner, the safety node
and the tests. Derivation with drawings: docs/design/kinematics.md.

The idea in three lines
-----------------------
* J1 (base yaw) just turns the whole arm, so look at the arm from the side:
  what remains is a flat 3-link arm (upper arm, forearm, hand) in the plane (r, z),
  where r = horizontal distance from the J1 axis and z = height.
* The pose of the hand is: where the grip centre is (x, y, z), which way the hand
  points in that plane ("pitch": 0 = straight up, 90 deg = horizontal, 180 deg =
  straight down) and how the gripper is rolled about the hand axis.
* IK: subtract the hand from the target to get the wrist point, then solve the
  classic two-link triangle (law of cosines) for shoulder and elbow.

Angle conventions are those of arm_params.json (zero pose = arm straight up).
All functions use radians and metres.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .robot import Robot


@dataclass
class Pose:
    x: float
    y: float
    z: float
    pitch: float     # direction of the hand in the vertical plane, from "up" towards "forward"
    roll: float      # J5

    def as_list(self):
        return [self.x, self.y, self.z, self.pitch, self.roll]


@dataclass
class IKResult:
    ok: bool
    q: list | None = None            # J1..J5 (rad); J6 is not part of IK
    reason: str = ""                 # why it failed, in plain words
    pitch_relaxed: bool = False      # the requested pitch was unreachable; a nearby pitch was used


def fk(robot: Robot, q) -> Pose:
    """Joint angles (J1..J5 or J1..J6) -> pose of the grip centre."""
    q1, q2, q3, q4, q5 = q[:5]
    a1, a2, a3 = q2, q2 + q3, q2 + q3 + q4          # absolute angles of the three links, from vertical
    r = robot.shoulder_offset + robot.l1 * math.sin(a1) + robot.l2 * math.sin(a2) + robot.l3 * math.sin(a3)
    z = robot.shoulder_height + robot.l1 * math.cos(a1) + robot.l2 * math.cos(a2) + robot.l3 * math.cos(a3)
    return Pose(r * math.cos(q1), r * math.sin(q1), z, a3, q5)


def joint_points(robot: Robot, q):
    """(r, z) of shoulder, elbow, wrist and grip centre in the arm plane (used for keep-out checks)."""
    q2, q3, q4 = q[1:4]
    a1, a2, a3 = q2, q2 + q3, q2 + q3 + q4
    pts = [(robot.shoulder_offset, robot.shoulder_height)]
    for length, a in ((robot.l1, a1), (robot.l2, a2), (robot.l3, a3)):
        pts.append((pts[-1][0] + length * math.sin(a), pts[-1][1] + length * math.cos(a)))
    return pts


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _solve_plane(robot: Robot, r, z, pitch):
    """Both elbow solutions (q2, q3, q4) for a hand at (r, z) pointing along `pitch`, or [] if out of reach."""
    rw = r - robot.shoulder_offset - robot.l3 * math.sin(pitch)      # wrist point = target minus the hand
    zw = z - robot.shoulder_height - robot.l3 * math.cos(pitch)
    d2 = rw * rw + zw * zw
    c3 = (d2 - robot.l1 ** 2 - robot.l2 ** 2) / (2 * robot.l1 * robot.l2)   # law of cosines
    if c3 > 1.0 + 1e-9 or c3 < -1.0 - 1e-9:
        return []
    c3 = max(-1.0, min(1.0, c3))
    out = []
    for q3 in (math.acos(c3), -math.acos(c3)):
        q2 = math.atan2(rw, zw) - math.atan2(robot.l2 * math.sin(q3), robot.l1 + robot.l2 * math.cos(q3))
        out.append((_wrap(q2), q3, _wrap(pitch - q2 - q3)))
    return out


def ik(robot: Robot, pose: Pose, q_near=None, allow_pitch_relax: bool = False) -> IKResult:
    """Pose -> joint angles J1..J5. Picks the solution inside the joint limits that is closest to q_near.

    Unreachable targets are REPORTED (ok=False with a reason), never silently clamped.
    With allow_pitch_relax the hand direction may deviate if the exact pitch is impossible
    (the "numeric fallback": a search over nearby pitch angles).
    """
    q_near = list(q_near) if q_near is not None else [0.0] * 5
    r = math.hypot(pose.x, pose.y)
    q1 = math.atan2(pose.y, pose.x) if r > 1e-6 else q_near[0]       # directly above the base: keep the yaw
    if not (robot.q_min[0] - 1e-9 <= q1 <= robot.q_max[0] + 1e-9):
        return IKResult(False, reason=f"base yaw {math.degrees(q1):.0f} deg is outside the J1 range")
    if not (robot.q_min[4] - 1e-9 <= pose.roll <= robot.q_max[4] + 1e-9):
        return IKResult(False, reason="roll is outside the J5 range")

    def best(pitch):
        sols = _solve_plane(robot, r, pose.z, pitch)
        if not sols:
            return None, "target is out of reach (too far or too close)"
        good = [s for s in sols if all(robot.q_min[i + 1] - 1e-9 <= s[i] <= robot.q_max[i + 1] + 1e-9 for i in range(3))]
        if not good:
            return None, "reachable only beyond the joint limits"
        return min(good, key=lambda s: sum((s[i] - q_near[i + 1]) ** 2 for i in range(3))), ""

    sol, why = best(pose.pitch)
    if sol:
        return IKResult(True, [q1, sol[0], sol[1], sol[2], pose.roll])
    if allow_pitch_relax:
        for step in range(1, 37):                                    # +-2.5 deg steps up to +-90 deg
            for sign in (1, -1):
                sol, _ = best(pose.pitch + sign * math.radians(2.5 * step))
                if sol:
                    return IKResult(True, [q1, sol[0], sol[1], sol[2], pose.roll], pitch_relaxed=True)
    return IKResult(False, reason=why)


def max_reach(robot: Robot) -> float:
    return robot.shoulder_offset + robot.l1 + robot.l2 + robot.l3
