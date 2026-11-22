"""Model of how one joint moves: the Python twin of firmware/lib/armcore/motion.h.

Role in the system
------------------
The simulator uses this so the simulated arm accelerates, cruises and brakes
exactly like the real firmware does; the safety node uses the same rule to
rate-limit what it passes on. If you change the rule, change BOTH files.
"""
from __future__ import annotations

import math


class JointMotion:
    def __init__(self, vmax, amax, q_min, q_max, pos=0.0):
        self.vmax, self.amax, self.q_min, self.q_max = vmax, amax, q_min, q_max
        self.pos = self.target = pos
        self.vel = 0.0
        self.speed_cap = 0.0
        self.clamped = False

    def reset(self, pos):
        self.pos = self.target = pos
        self.vel = 0.0

    def set_target(self, t, cap=0.0):
        self.clamped = t < self.q_min - 1e-9 or t > self.q_max + 1e-9
        self.target = min(max(t, self.q_min), self.q_max)
        self.speed_cap = cap if 0 < cap < self.vmax else 0.0

    def stop(self):
        d = self.vel * self.vel / (2 * self.amax)
        self.target = self.pos + (d if self.vel >= 0 else -d)

    def moving(self):
        return abs(self.target - self.pos) > 3.5e-4 or abs(self.vel) > 8.7e-3

    def step(self, dt):
        err = self.target - self.pos
        vlim = self.speed_cap or self.vmax
        # fastest speed from which we can still stop in time. This is the exact answer for braking in
        # steps of dt (the continuous formula sqrt(2*a*err) brakes a little late and overshoots).
        adt = self.amax * dt
        want = min(vlim, adt * (math.sqrt(0.25 + 2 * abs(err) / (adt * dt)) - 0.5))
        if err < 0:
            want = -want
        dv = max(-self.amax * dt, min(self.amax * dt, want - self.vel))
        self.vel += dv
        step = self.vel * dt
        if abs(err) <= abs(step) and abs(self.vel) <= self.amax * dt * 2:
            self.pos, self.vel = self.target, 0.0
        else:
            self.pos += step
