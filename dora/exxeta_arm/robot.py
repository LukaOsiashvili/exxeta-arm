"""Robot model: geometry, joint limits and named poses, loaded from the single source of truth.

Role in the system
------------------
Every PC-side module (kinematics, planner, safety, simulator) gets its numbers
from here, and this file reads them from cad/params/arm_params.json, the same
file that drives the CAD. Internally everything is SI: metres, radians, seconds.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(os.environ.get("EXXETA_ARM_ROOT", Path(__file__).resolve().parents[2]))
PARAMS_PATH = PROJECT_ROOT / "cad" / "params" / "arm_params.json"
JOINT_IDS = ["J1", "J2", "J3", "J4", "J5", "J6"]
N = 6


@dataclass
class Robot:
    names: list
    q_min: list            # rad
    q_max: list
    v_max: list            # rad/s  (default conservative caps)
    a_max: list            # rad/s^2
    has_pot: list
    shoulder_height: float  # m
    shoulder_offset: float
    l1: float              # upper arm
    l2: float              # forearm
    l3: float              # wrist axis -> grip centre (TCP)
    gripper_m_per_rad: float
    base_radius: float     # keep-out cylinder around the base
    base_height: float
    payload_kg: float
    poses: dict = field(default_factory=dict)   # named poses, rad

    def clamp(self, q):
        return [min(max(v, lo), hi) for v, lo, hi in zip(q, self.q_min, self.q_max)]

    def within_limits(self, q, tol=1e-6):
        return all(lo - tol <= v <= hi + tol for v, lo, hi in zip(q, self.q_min, self.q_max))


def _strip(node):
    if isinstance(node, dict):
        return {k: _strip(v) for k, v in node.items() if not k.startswith("_")}
    return node


def load_robot(path: Path | str = PARAMS_PATH) -> Robot:
    p = _strip(json.loads(Path(path).read_text(encoding="utf-8")))
    j = [p["joints"][k] for k in JOINT_IDS]
    g, jd = p["geometry"], p["joint_design"]
    rad = math.radians
    pinion_r = jd["gripper_pinion_module"] * jd["gripper_pinion_teeth"] / 2 / 1000.0
    deg_poses = {                       # degrees, same as cad/assembly.py POSES
        "rest": (0, -72, 120, 85, 0, 0),
        "home": (0, 20, 70, 0, 0, 90),
        "ready": (0, 10, 60, 50, 0, 90),
        "zero": (0, 0, 0, 0, 0, 0),
    }
    return Robot(
        names=[x["name"] for x in j],
        q_min=[rad(x["min_deg"]) for x in j], q_max=[rad(x["max_deg"]) for x in j],
        v_max=[rad(x["max_speed_deg_s"]) for x in j], a_max=[rad(x["max_accel_deg_s2"]) for x in j],
        has_pot=[bool(x["pot"]) for x in j],
        shoulder_height=g["shoulder_height_mm"] / 1000, shoulder_offset=g["shoulder_offset_mm"] / 1000,
        l1=g["upper_arm_length_mm"] / 1000, l2=g["forearm_length_mm"] / 1000, l3=g["wrist_to_tcp_mm"] / 1000,
        gripper_m_per_rad=pinion_r,
        base_radius=max(p["base"]["footprint_mm"]) / 2 / 1000 * 1.25,   # circle around the square base + margin
        base_height=p["base"]["height_mm"] / 1000,
        payload_kg=p["design_targets"]["payload_g"] / 1000,
        poses={k: [rad(a) for a in v] for k, v in deg_poses.items()},
    )
