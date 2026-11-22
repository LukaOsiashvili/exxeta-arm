"""Printed parts of exxeta-arm. One module per part; each exposes `PART` (a PartDef)."""
import importlib

# Order = part numbers on the STL files.
MODULES = [
    "base_shell", "base_panel", "j1_bracket", "clamp_ring", "turntable",
    "upper_arm", "forearm", "wrist_block", "wrist_cap", "gripper_body", "gripper_finger", "gripper_pinion",
    "fingertip", "pivot_pin", "pivot_cover", "pot_pod", "pot_coupler", "j1_pot_gear",
    "tolerance_test", "test_rig_stand", "test_rig_arm",
]


def all_parts(only=None):
    """Import the part modules and return their PartDefs (skipping modules not written yet)."""
    out = []
    for name in MODULES:
        if only and name not in only:
            continue
        try:
            out.append(importlib.import_module(f"parts.{name}").PART)
        except ModuleNotFoundError as e:
            if e.name != f"parts.{name}":
                raise
    return out
