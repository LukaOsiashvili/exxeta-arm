"""Safety gate: the last check on the PC before anything reaches the arm.

Role in the system
------------------
Pure logic behind the `safety` node (PROJECT_SPEC 8.2 and 10). Whatever the
planner, a recording or a buggy exercise node asks for, this module
  * keeps every joint inside its limits,
  * limits speed and acceleration (same rule as the firmware),
  * refuses poses that would put the hand through the table or into the base,
  * runs the mode state machine  DISABLED / IDLE / MOVING / STOPPED / FAULT,
  * latches STOP until it is acknowledged,
  * refuses motion when the battery is critical or the link is lost.
The firmware enforces limits again on its own: two independent layers.
"""
from __future__ import annotations

from . import kinematics as kin
from .protocol import Fault, Mode
from .robot import Robot
from .servo_model import JointMotion

TABLE_CLEARANCE_M = 0.005      # nothing of the hand below this height
HAND_RADIUS_M = 0.03           # the hand is treated as a fat line from wrist to fingertip


class Safety:
    def __init__(self, robot: Robot, speed_scale: float = 1.0):
        self.robot = robot
        self.speed_scale = speed_scale
        self.mode = Mode.DISABLED
        self.stop_latched = False
        self.device_mode = Mode.DISABLED
        self.device_faults = 0
        self.connected = False
        self.reason = ""                        # why the last goal was changed or refused
        self.motion = [JointMotion(v, a, lo, hi, p) for v, a, lo, hi, p in
                       zip(robot.v_max, robot.a_max, robot.q_min, robot.q_max, robot.poses["rest"])]
        self._last_safe = list(robot.poses["rest"])

    # ---------------------------------------------------------------- inputs
    def on_device_status(self, connected: bool, mode: int, faults: int):
        self.connected, self.device_mode, self.device_faults = connected, mode, faults
        if not connected or mode == Mode.DISABLED:
            self.mode = Mode.DISABLED
        elif faults & (Fault.LINK_LOST | Fault.BATTERY_CRITICAL | Fault.SERVO_POWER_OFF):
            self.mode = Mode.FAULT
        elif self.stop_latched or faults & Fault.STOP_LATCHED:
            self.mode = Mode.STOPPED
        elif self.mode in (Mode.DISABLED, Mode.FAULT, Mode.STOPPED):
            self.mode = Mode.IDLE

    def sync(self, q):
        """Start from the arm's real position (called when the arm becomes enabled or after a clear)."""
        for m, p in zip(self.motion, q):
            m.reset(p)
        self._last_safe = list(q)

    def stop(self):
        """Controlled stop: brake every joint, then hold. Latches until clear()."""
        self.stop_latched = True
        for m in self.motion:
            m.stop()
        if self.mode != Mode.DISABLED:
            self.mode = Mode.STOPPED

    def clear(self):
        self.stop_latched = False
        for m in self.motion:
            m.target = m.pos
        if self.mode == Mode.STOPPED:
            self.mode = Mode.IDLE

    def can_move(self) -> bool:
        return self.mode in (Mode.IDLE, Mode.MOVING) and not self.stop_latched

    # ----------------------------------------------------------------- checks
    def check_workspace(self, q) -> str:
        """'' if the pose is allowed, else the reason. Table plane, base cylinder, simple self-collision."""
        r = self.robot
        pts = kin.joint_points(r, q)
        (rw, zw), (rt, zt) = pts[2], pts[3]
        # fingertips are 15 mm beyond the grip centre
        ext = 1.0 + 0.015 / r.l3
        rt, zt = rw + (rt - rw) * ext, zw + (zt - zw) * ext
        for k in range(5):                                   # sample the hand from wrist to fingertip
            s = k / 4
            pr, pz = rw + s * (rt - rw), zw + s * (zt - zw)
            if pz < TABLE_CLEARANCE_M + (HAND_RADIUS_M if k < 4 else 0.0):
                return "hand would hit the table"
            if abs(pr) < r.base_radius and pz < r.base_height + 0.012 + HAND_RADIUS_M:
                return "hand would hit the base"
            if abs(pr) < 0.03 and pz < r.shoulder_height + 0.03:
                return "hand would hit the shoulder"
        er, ez = pts[1]
        if ez < r.base_height + 0.035 and abs(er) < r.base_radius:
            return "elbow would hit the base"
        return ""

    # ------------------------------------------------------------------- tick
    def filter(self, q_goal, dt: float):
        """Take the planner's goal, return the safe target to send (always a valid pose)."""
        self.reason = ""
        if self.can_move() and q_goal is not None:
            q = self.robot.clamp(q_goal)
            if q != list(q_goal):
                self.reason = "goal clamped to the joint limits"
            why = self.check_workspace(q)
            if why:
                self.reason = why + ": holding"
            else:
                for m, t in zip(self.motion, q):
                    m.set_target(t)
        moving = False
        for m in self.motion:
            vm = m.vmax
            m.vmax = vm * self.speed_scale
            m.step(dt)
            m.vmax = vm
            moving |= m.moving()
        out = [m.pos for m in self.motion]
        if self.mode in (Mode.IDLE, Mode.MOVING):
            self.mode = Mode.MOVING if moving else Mode.IDLE
        if self.check_workspace(out):
            # never emit a pose inside a keep-out zone: freeze on the last good one
            for m, p in zip(self.motion, self._last_safe):
                m.reset(p)
            out = list(self._last_safe)
        else:
            self._last_safe = out
        return out
