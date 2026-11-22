"""Shared loader for cad/params/arm_params.json.

Role in the system
------------------
`arm_params.json` is the single source of truth for geometry, servo choices and
mass estimates. Every tool (torque budget, power budget, CAD, URDF generator)
loads it through this module so that they all see exactly the same numbers.

Only the Python standard library is used, so the tools run anywhere.
"""
from __future__ import annotations

import json
from pathlib import Path

# <project root>/tools/arm_params.py  ->  <project root>
PROJECT_ROOT = Path(__file__).resolve().parent.parent
PARAMS_PATH = PROJECT_ROOT / "cad" / "params" / "arm_params.json"

G = 9.80665  # standard gravity, m/s^2
KGCM_TO_NM = G / 100.0  # 1 kgf*cm = 0.0980665 N*m


def _strip_comments(node):
    """Remove every key that starts with '_' (those are comments in the JSON)."""
    if isinstance(node, dict):
        return {k: _strip_comments(v) for k, v in node.items() if not k.startswith("_")}
    if isinstance(node, list):
        return [_strip_comments(v) for v in node]
    return node


def load_params(path: Path | str = PARAMS_PATH) -> dict:
    """Load the parameter file and drop the comment keys."""
    with open(path, encoding="utf-8") as f:
        return _strip_comments(json.load(f))


def servo_torque_kgcm(servo: dict, voltage: float) -> float:
    """Rated stall torque at `voltage`, linearly interpolated between 4.8 V and 6 V.

    Hobby-servo torque is roughly proportional to supply voltage, so a straight
    line through the two datasheet points is a fair model between ~4.5 and 6.5 V.
    """
    t48, t60 = servo["stall_torque_kgcm_4v8"], servo["stall_torque_kgcm_6v"]
    return t48 + (t60 - t48) * (voltage - 4.8) / (6.0 - 4.8)
