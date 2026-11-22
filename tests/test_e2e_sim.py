"""End-to-end test of the simulation dataflow, without dora and without a browser.

All eight nodes are created from dataflows/sim.yml by tools/run_local.py and run
on virtual time. Commands enter exactly where the web app's would (the queue of
the web_server node).
"""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "dora"))

pytest.importorskip("aiohttp")
from run_local import LocalDataflow  # noqa: E402

from exxeta_arm.robot import load_robot  # noqa: E402

ROBOT = load_robot()


@pytest.fixture()
def flow(tmp_path, monkeypatch):
    import exxeta_arm.runtime as rt
    monkeypatch.setattr(rt, "load_config", lambda: {"web": {"port": 18080 + (id(tmp_path) % 500)}, "sim": {"auto_connect": True}})
    f = LocalDataflow(ROOT / "dora" / "dataflows" / "sim.yml")
    f.ws = f.nodes["web_server"]
    f.cmd = lambda d: f.ws.to_dora.put(("ui_command", d))
    f.run_virtual(0.3)
    yield f
    for n in f.nodes.values():
        n.close()
    rt.set_clock(__import__("time").monotonic)


def q_deg(f):
    return [math.degrees(a) for a in f.ws.state["estimated_state"]["q"]]


def near(a, b, tol):
    return max(abs(x - y) for x, y in zip(a, b)) < tol


def test_enable_move_stop_clear_park(flow):
    f = flow
    assert f.ws.state["safety_status"]["mode_name"] == "DISABLED"
    f.cmd({"cmd": "move_pose", "name": "home"})            # ignored while disabled
    f.run_virtual(1.0)
    assert "not enabled" in f.ws.state["planner_status"]["message"]

    f.cmd({"cmd": "enable"})
    f.run_virtual(0.3)
    assert f.ws.state["safety_status"]["mode_name"] == "IDLE"
    assert near(q_deg(f), [0, -72, 120, 85, 0, 0], 0.5)     # starts in the rest pose, no jump

    f.cmd({"cmd": "set_speed", "scale": 1.0})
    f.cmd({"cmd": "move_pose", "name": "home"})
    f.run_virtual(8.0)
    assert near(q_deg(f), [0, 20, 70, 0, 0, 90], 0.5)

    f.cmd({"cmd": "move_pose", "name": "ready"})
    f.run_virtual(0.5)
    f.cmd({"cmd": "stop"})                                  # STOP in mid-move
    f.run_virtual(1.5)
    assert f.ws.state["safety_status"]["mode_name"] == "STOPPED"
    held = q_deg(f)
    f.cmd({"cmd": "move_pose", "name": "ready"})            # refused while STOP is latched
    f.run_virtual(1.0)
    assert near(q_deg(f), held, 0.2)

    f.cmd({"cmd": "clear"})
    f.run_virtual(0.3)
    assert f.ws.state["safety_status"]["mode_name"] == "IDLE"

    f.cmd({"cmd": "park"})                                  # Park & power off
    f.run_virtual(10.0)
    assert f.ws.state["safety_status"]["mode_name"] == "DISABLED"
    assert near(q_deg(f), [0, -72, 120, 85, 0, 0], 0.5)


def test_cartesian_move_and_ee_pose(flow):
    f = flow
    f.cmd({"cmd": "enable"})
    f.cmd({"cmd": "set_speed", "scale": 1.0})
    f.run_virtual(0.3)
    f.cmd({"cmd": "move_pose", "name": "ready"})
    f.run_virtual(8.0)
    p = f.ws.state["ee_pose"]
    goal = [p["x"], p["y"] + 0.05, p["z"] - 0.02, p["pitch"], 0.0]
    f.cmd({"cmd": "move_cartesian", "pose": goal})
    f.run_virtual(8.0)
    p2 = f.ws.state["ee_pose"]
    assert math.dist((p2["x"], p2["y"], p2["z"]), goal[:3]) < 0.001, f.ws.state["planner_status"]["message"]


def test_record_and_replay_program(flow, tmp_path, monkeypatch):
    f = flow
    rec = f.nodes["recorder"]
    mod = sys.modules[type(rec).__module__]
    monkeypatch.setattr(mod, "REC_DIR", tmp_path)
    home, ready = ROBOT.poses["home"], ROBOT.poses["ready"]
    f.cmd({"cmd": "program_save", "name": "t1", "steps": [
        {"type": "joint", "q": home, "speed": 1.0, "dwell": 0.2},
        {"type": "joint", "q": ready, "speed": 1.0, "gripper": 0.5, "dwell": 0.0}]})
    f.cmd({"cmd": "enable"})
    f.run_virtual(0.5)
    assert (tmp_path / "t1.json").exists()
    f.cmd({"cmd": "play", "name": "t1", "speed": 1.0})
    f.run_virtual(25.0)
    assert f.ws.state["recorder_status"]["playing"] == ""
    q = f.ws.state["estimated_state"]["q"]
    assert near(q[:5], ready[:5], 0.01) and abs(q[5] - 0.5) < 0.01
