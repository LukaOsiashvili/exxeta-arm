"""Motion planner: turns "go there" into a smooth stream of joint goals.

Role in the system
------------------
Pure logic behind the `planner` node. Every 20 ms `step(dt)` returns the next
joint goal (6 angles, rad). It supports
  * joint moves      : all joints start and stop together, minimum-jerk profile
  * Cartesian lines  : the grip centre moves on a straight line (IK at every sample,
                       rejected up front if any point is unreachable or the joints would jump)
  * jogging          : continuous joint or Cartesian velocity while a button is held
  * named poses and the gripper.
Nothing here talks to hardware: the output still passes the safety node and the firmware limits.
"""
from __future__ import annotations

import math

from . import kinematics as kin
from .robot import Robot

JOG_TIMEOUT_S = 0.3        # a jog command must be repeated; otherwise motion stops (dropped connection = stop)


def min_jerk(s: float) -> float:
    """Smooth 0..1 blend with zero speed and acceleration at both ends."""
    s = min(max(s, 0.0), 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class Planner:
    def __init__(self, robot: Robot, speed_scale: float = 0.5):
        self.robot = robot
        self.speed_scale = speed_scale          # 0..1, set from the UI (conservative default, PROJECT_SPEC 10.7)
        self.q = list(robot.poses["rest"])      # current goal (what we last sent)
        self.synced = False                     # becomes True once we know where the arm is
        self._traj = None                       # (list of waypoints, duration, elapsed)
        self._jog = None                        # ("joint"|"cart", vector, age)
        self.status = "idle"
        self.message = ""

    # ------------------------------------------------------------------ state
    def sync(self, q):
        """Adopt the arm's actual position as the starting point (on enable, after a stop)."""
        self.q = list(q)
        self._traj = self._jog = None
        self.synced = True
        self.status = "idle"

    def busy(self):
        return self._traj is not None or self._jog is not None

    def cancel(self):
        self._traj = self._jog = None
        self.status = "idle"

    # ------------------------------------------------------------------ goals
    def _duration(self, q_from, q_to, speed=None):
        """Time so that no joint exceeds its (scaled) speed limit; 1.875 = peak/mean speed of min-jerk."""
        scale = self.speed_scale if speed is None else speed
        t = 0.2
        for a, b, v, acc in zip(q_from, q_to, self.robot.v_max, self.robot.a_max):
            d = abs(b - a)
            t = max(t, 1.875 * d / (v * scale), math.sqrt(5.774 * d / acc))
        return t

    def move_joints(self, q_goal, speed=None) -> bool:
        q_goal = list(q_goal)
        if len(q_goal) == 5:
            q_goal.append(self.q[5])
        if not self.robot.within_limits(q_goal):
            self.message = "goal is outside the joint limits"
            return False
        self._jog = None
        self._traj = ([list(self.q), q_goal], self._duration(self.q, q_goal, speed), 0.0)
        self.status, self.message = "moving", ""
        return True

    def move_pose(self, name: str, speed=None) -> bool:
        if name not in self.robot.poses:
            self.message = f"unknown pose '{name}'"
            return False
        return self.move_joints(self.robot.poses[name], speed)

    def move_cartesian(self, pose: kin.Pose, speed=None, step_m=0.003) -> bool:
        """Straight line of the grip centre from the current pose to `pose`."""
        start = kin.fk(self.robot, self.q)
        dist = math.dist((start.x, start.y, start.z), (pose.x, pose.y, pose.z))
        n = max(2, int(dist / step_m) + 1, int(abs(pose.pitch - start.pitch) / 0.03) + 1)
        pts, q_prev = [list(self.q)], list(self.q)
        for i in range(1, n + 1):
            s = i / n
            p = kin.Pose(start.x + s * (pose.x - start.x), start.y + s * (pose.y - start.y), start.z + s * (pose.z - start.z),
                         start.pitch + s * (pose.pitch - start.pitch), start.roll + s * (pose.roll - start.roll))
            res = kin.ik(self.robot, p, q_prev)
            if not res.ok:
                self.message = f"line not possible at {int(s * 100)} %: {res.reason}"
                return False
            if max(abs(a - b) for a, b in zip(res.q, q_prev)) > 0.35:      # continuity check (about 20 degrees)
                self.message = f"line not possible at {int(s * 100)} %: the joints would have to jump"
                return False
            q_prev = res.q + [self.q[5]]
            pts.append(q_prev)
        duration = max(self._duration(self.q, pts[-1], speed), max(
            self._duration(a, b, speed) for a, b in zip(pts, pts[1:])) * 0.35 * n)
        self._jog = None
        self._traj = (pts, duration, 0.0)
        self.status, self.message = "moving", ""
        return True

    def set_gripper(self, angle, speed=None) -> bool:
        q = list(self._traj[0][-1]) if self._traj else list(self.q)
        q[5] = min(max(angle, self.robot.q_min[5]), self.robot.q_max[5])
        return self.move_joints(q, speed)

    def jog_joint(self, vel):
        """vel: 6 values in -1..1 (fraction of each joint's scaled speed limit)."""
        self._traj = None
        self._jog = ("joint", [max(-1.0, min(1.0, v)) for v in vel], 0.0)
        self.status = "jogging"

    def jog_cartesian(self, vel):
        """vel: [vx, vy, vz, vpitch, vroll, vgrip] in -1..1 (fractions of 0.08 m/s, 1 rad/s and the J5/J6 limits)."""
        self._traj = None
        self._jog = ("cart", [max(-1.0, min(1.0, v)) for v in vel], 0.0)
        self.status = "jogging"

    # ------------------------------------------------------------------- tick
    def step(self, dt: float):
        """Advance by dt and return the joint goal to send now."""
        if self._traj:
            pts, duration, t = self._traj
            t += dt
            u = min_jerk(t / duration) * (len(pts) - 1)       # position along the list of waypoints
            i = min(int(u), len(pts) - 2)
            f = u - i
            self.q = [a + f * (b - a) for a, b in zip(pts[i], pts[i + 1])]
            self._traj = (pts, duration, t) if t < duration else None
            if self._traj is None:
                self.q, self.status = list(pts[-1]), "idle"
        elif self._jog:
            kind, vel, age = self._jog
            age += dt
            if age > JOG_TIMEOUT_S:
                self._jog, self.status = None, "idle"
            else:
                self._jog = (kind, vel, age)
                (self._step_jog_joint if kind == "joint" else self._step_jog_cart)(vel, dt)
        return list(self.q)

    def _step_jog_joint(self, vel, dt):
        r = self.robot
        self.q = [min(max(q + v * vm * self.speed_scale * dt, lo), hi)
                  for q, v, vm, lo, hi in zip(self.q, vel, r.v_max, r.q_min, r.q_max)]

    def _step_jog_cart(self, vel, dt):
        r, s = self.robot, self.speed_scale
        p = kin.fk(r, self.q)
        target = kin.Pose(p.x + vel[0] * 0.08 * s * dt, p.y + vel[1] * 0.08 * s * dt, p.z + vel[2] * 0.08 * s * dt,
                          p.pitch + vel[3] * 1.0 * s * dt, p.roll + vel[4] * r.v_max[4] * s * dt)
        res = kin.ik(r, target, self.q)
        if res.ok and max(abs(a - b) for a, b in zip(res.q, self.q)) < 0.2:
            grip = min(max(self.q[5] + vel[5] * r.v_max[5] * s * dt, r.q_min[5]), r.q_max[5])
            self.q = res.q + [grip]
            self.message = ""
        else:
            self.message = res.reason or "jog blocked: joints would jump"   # stay where we are at the workspace edge
