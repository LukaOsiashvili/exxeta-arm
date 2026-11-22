"""Simulated arm: behaves like the ESP32 firmware, without any hardware.

Role in the system
------------------
The `sim_bridge` node wraps this class. It accepts the same commands as the real
arm (enable, targets, stop, ...) and produces the same State messages, so the
whole stack, including the web app, runs on any laptop (PROJECT_SPEC 9).
Servos follow the same speed/acceleration rule as the firmware. The battery
drains slowly so the low-battery behaviour can be tried out; pots can be faked.
"""
from __future__ import annotations

import math
import random

from .protocol import Fault, Mode, State
from .robot import Robot
from .servo_model import JointMotion


class SimArm:
    def __init__(self, robot: Robot, with_pots: bool = False, battery_v: float = 8.1, watchdog_s: float = 0.5):
        self.robot = robot
        self.with_pots = with_pots
        self.battery_v = battery_v
        self.watchdog_s = watchdog_s
        self.mode = Mode.DISABLED
        self.faults = 0
        self.joints = [JointMotion(v * 3, a * 3, lo, hi, p) for v, a, lo, hi, p in   # firmware caps are the servo's,
                       zip(robot.v_max, robot.a_max, robot.q_min, robot.q_max, robot.poses["rest"])]  # looser than the PC's
        self.t = 0.0
        self._since_cmd = 0.0
        self._parking = False

    # ----------------------------------------------------------- commands
    def touch(self):
        """Any valid message from the PC feeds the watchdog."""
        self._since_cmd = 0.0

    def enable(self) -> bool:
        if self.faults & (Fault.LINK_LOST | Fault.STOP_LATCHED | Fault.BATTERY_CRITICAL):
            return False
        if self.mode == Mode.DISABLED:
            for j, p in zip(self.joints, self.robot.poses["rest"]):     # no feedback: assume the rest pose
                j.reset(p)
            self.mode = Mode.IDLE
        return True

    def disable(self):
        self.mode = Mode.DISABLED
        self._parking = False

    def set_targets(self, q, speeds=None):
        if self.mode in (Mode.IDLE, Mode.MOVING) and not self._parking and \
                not self.faults & (Fault.LINK_LOST | Fault.STOP_LATCHED | Fault.BATTERY_CRITICAL):
            clamped = False
            for i, (j, t) in enumerate(zip(self.joints, q)):
                j.set_target(t, speeds[i] if speeds else 0.0)
                clamped |= j.clamped
            self.faults = self.faults | Fault.LIMIT_CLAMPED if clamped else self.faults & ~Fault.LIMIT_CLAMPED

    def stop(self):
        for j in self.joints:
            j.stop()
        if self.mode != Mode.DISABLED:
            self.mode = Mode.STOPPED
        self.faults |= Fault.STOP_LATCHED

    def clear(self):
        self.faults &= ~(Fault.STOP_LATCHED | Fault.LINK_LOST | Fault.LIMIT_CLAMPED)
        for j in self.joints:
            j.target = j.pos
        if self.mode in (Mode.STOPPED, Mode.FAULT):
            self.mode = Mode.IDLE

    # --------------------------------------------------------------- tick
    def step(self, dt: float):
        self.t += dt
        self._since_cmd += dt
        if self._since_cmd > self.watchdog_s and self.mode != Mode.DISABLED and not self.faults & Fault.LINK_LOST:
            for j in self.joints:                      # link watchdog: hold position, latch LINK_LOST
                j.stop()
            self.faults |= Fault.LINK_LOST
        # battery: slow drain while enabled, with the firmware's thresholds
        if self.mode != Mode.DISABLED:
            self.battery_v -= dt * 0.00004
        self.faults = self.faults | Fault.BATTERY_WARN if self.battery_v < 7.0 else self.faults & ~Fault.BATTERY_WARN
        if self.battery_v < 6.6:
            self.faults |= Fault.BATTERY_CRITICAL
            if self.mode != Mode.DISABLED and not self._parking:
                self._parking = True
                for j, p in zip(self.joints, self.robot.poses["rest"]):
                    j.set_target(p, math.radians(30))
        if self.mode != Mode.DISABLED:
            moving = False
            for j in self.joints:
                j.step(dt)
                moving |= j.moving()
            if self.mode in (Mode.IDLE, Mode.MOVING):
                self.mode = Mode.MOVING if moving else Mode.IDLE

    def state(self) -> State:
        deg = math.degrees
        s = State(mode=self.mode, faults=self.faults,
                  target_deg=[deg(j.target) for j in self.joints], position_deg=[deg(j.pos) for j in self.joints],
                  battery_v=self.battery_v, battery_pct=int(max(0, min(100, (self.battery_v / 2 - 3.3) / 0.8 * 100))),
                  uptime_ms=int(self.t * 1000))
        if self.with_pots:
            for i in range(4):
                if self.robot.has_pot[i]:
                    s.pot_deg[i] = deg(self.joints[i].pos) + random.gauss(0, 0.4)   # pots are a little noisy
                    s.pot_valid |= 1 << i
        return s
