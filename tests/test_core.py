"""Tests of the PC-side core library (no dora, no hardware needed).

Run from the project root:   python -m pytest tests -q
"""
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "dora"))

from exxeta_arm import kinematics as kin  # noqa: E402
from exxeta_arm import protocol as pr  # noqa: E402
from exxeta_arm.planner import Planner  # noqa: E402
from exxeta_arm.protocol import Fault, Mode  # noqa: E402
from exxeta_arm.robot import load_robot  # noqa: E402
from exxeta_arm.safety import Safety  # noqa: E402
from exxeta_arm.sim import SimArm  # noqa: E402

ROBOT = load_robot()
DT = 0.02


# ------------------------------------------------------------------ protocol
def test_protocol_vectors_match_python_encoder():
    """The shared vector file is what the firmware tests use: it must match today's Python code."""
    data = json.loads((ROOT / "tests" / "protocol_vectors.json").read_text())
    assert data["protocol_version"] == pr.VERSION
    for v in data["vectors"]:
        f = pr.Frame(v["type"], v["seq"], bytes.fromhex(v["payload_hex"]))
        raw = pr.encode_frame(f)
        assert raw.hex() == v["frame_hex"], v["name"]
        assert pr.cobs_encode(raw).hex() == v["cobs_hex"], v["name"]
        back = pr.decode_frame(pr.cobs_decode(bytes.fromhex(v["cobs_hex"])))
        assert (back.type, back.seq, back.payload) == (f.type, f.seq, f.payload)


def test_cobs_roundtrip_random():
    rnd = random.Random(1)
    for n in (0, 1, 5, 253, 254, 255, 300, 600):
        data = bytes(rnd.choice([0, 0, 1, 255, rnd.randrange(256)]) for _ in range(n))
        enc = pr.cobs_encode(data)
        assert 0 not in enc
        assert pr.cobs_decode(enc) == data


def test_bad_crc_rejected():
    raw = bytearray(pr.encode_frame(pr.Frame(pr.Msg.STOP, 9)))
    raw[1] ^= 1
    try:
        pr.decode_frame(bytes(raw))
        assert False, "corrupted frame accepted"
    except ValueError:
        pass


def test_serial_deframer_separates_frames_and_text():
    d = pr.SerialDeframer()
    stream = b"# hello from the CLI\n" + pr.to_serial(pr.Frame(pr.Msg.HEARTBEAT, 3)) + b"# bye\n" + \
        pr.to_serial(pr.Frame(pr.Msg.STATE, 4, pr.pack_state(pr.State(mode=2, battery_v=7.5))))
    frames = []
    for i in range(0, len(stream), 7):            # arrives in arbitrary chunks
        frames += d.feed(stream[i:i + 7])
    assert [f.type for f in frames] == [pr.Msg.HEARTBEAT, pr.Msg.STATE]
    assert d.text_lines == ["# hello from the CLI", "# bye"]
    assert pr.parse_state(frames[1].payload).battery_v == 7.5


def test_state_roundtrip():
    s = pr.State(mode=1, faults=5, target_deg=[1, 2, 3, 4, 5, 6], position_deg=[-1.25, 0, 0, 0, 0, 160],
                 pot_deg=[1, 2, 3, 4], pot_valid=3, battery_v=7.123, battery_pct=40, uptime_ms=99, rx_ok=1, rx_bad=2, last_seq=3)
    assert pr.parse_state(pr.pack_state(s)) == s
    assert pr.STATE_SIZE == 49


# ---------------------------------------------------------------- kinematics
def test_fk_zero_pose_points_straight_up():
    p = kin.fk(ROBOT, [0] * 6)
    assert abs(p.x) < 1e-9 and abs(p.y) < 1e-9
    assert abs(p.z - (ROBOT.shoulder_height + ROBOT.l1 + ROBOT.l2 + ROBOT.l3)) < 1e-9


def test_fk_horizontal_reach():
    p = kin.fk(ROBOT, [0, math.pi / 2, 0, 0, 0, 0])
    assert abs(p.x - 0.255) < 1e-9 and abs(p.z - ROBOT.shoulder_height) < 1e-9


def test_ik_fk_roundtrip_over_sampled_workspace():
    """Acceptance criterion 4: every pose produced by valid joint angles is solved back exactly."""
    rnd = random.Random(42)
    for _ in range(3000):
        q = [rnd.uniform(lo, hi) for lo, hi in zip(ROBOT.q_min[:5], ROBOT.q_max[:5])]
        pose = kin.fk(ROBOT, q)
        if math.hypot(pose.x, pose.y) < 0.005:
            continue                                  # on the J1 axis the yaw is arbitrary
        res = kin.ik(ROBOT, pose, q)
        if not res.ok:
            # legal only if the grip centre is "behind" the base: atan2 then picks a yaw outside J1's range
            assert "yaw" in res.reason or "limits" in res.reason, res.reason
            continue
        back = kin.fk(ROBOT, res.q)
        assert math.dist((pose.x, pose.y, pose.z), (back.x, back.y, back.z)) < 1e-9
        assert abs(math.remainder(pose.pitch - back.pitch, 2 * math.pi)) < 1e-9
        assert ROBOT.within_limits(res.q + [0])


def test_ik_reports_unreachable():
    res = kin.ik(ROBOT, kin.Pose(0.5, 0, 0.1, math.pi / 2, 0))
    assert not res.ok and "reach" in res.reason
    res = kin.ik(ROBOT, kin.Pose(-0.15, 0.0, 0.15, math.pi / 2, 0))
    assert not res.ok and "yaw" in res.reason


def test_ik_pitch_relax_finds_nearby_solution():
    far = kin.Pose(0.22, 0, 0.12, math.radians(180), 0)       # pointing straight down that far out: impossible
    assert not kin.ik(ROBOT, far).ok
    res = kin.ik(ROBOT, far, allow_pitch_relax=True)
    assert res.ok and res.pitch_relaxed
    back = kin.fk(ROBOT, res.q)
    assert math.dist((far.x, far.y, far.z), (back.x, back.y, back.z)) < 1e-9


# ------------------------------------------------------------------- planner
def _run(planner, seconds=20.0):
    path, t = [], 0.0
    while t < seconds:
        path.append(planner.step(DT))
        t += DT
        if not planner.busy():
            break
    return path


def test_joint_move_reaches_goal_within_speed_limits():
    pl = Planner(ROBOT, speed_scale=0.5)
    pl.sync(ROBOT.poses["rest"])
    assert pl.move_pose("home")
    path = _run(pl)
    assert max(abs(a - b) for a, b in zip(path[-1], ROBOT.poses["home"])) < 1e-9
    for a, b in zip(path, path[1:]):
        for i in range(6):
            assert abs(b[i] - a[i]) / DT <= ROBOT.v_max[i] * 0.5 * 1.02


def test_move_outside_limits_is_refused():
    pl = Planner(ROBOT)
    pl.sync(ROBOT.poses["home"])
    assert not pl.move_joints([0, 3.0, 0, 0, 0, 0])
    assert "limits" in pl.message


def test_cartesian_move_is_a_straight_line():
    pl = Planner(ROBOT)
    pl.sync(ROBOT.poses["ready"])
    start = kin.fk(ROBOT, pl.q)
    goal = kin.Pose(start.x, start.y + 0.06, start.z - 0.03, start.pitch, 0.0)
    assert pl.move_cartesian(goal), pl.message
    path = _run(pl)
    end = kin.fk(ROBOT, path[-1])
    assert math.dist((end.x, end.y, end.z), (goal.x, goal.y, goal.z)) < 1e-6
    ax, ay, az = goal.x - start.x, goal.y - start.y, goal.z - start.z
    length = math.sqrt(ax * ax + ay * ay + az * az)
    for q in path:                                    # distance of every sample from the line
        p = kin.fk(ROBOT, q)
        bx, by, bz = p.x - start.x, p.y - start.y, p.z - start.z
        cx, cy, cz = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
        assert math.sqrt(cx * cx + cy * cy + cz * cz) / length < 0.0005   # 0.5 mm


def test_unreachable_cartesian_move_is_refused_with_reason():
    pl = Planner(ROBOT)
    pl.sync(ROBOT.poses["ready"])
    assert not pl.move_cartesian(kin.Pose(0.6, 0, 0.1, math.pi / 2, 0))
    assert "not possible" in pl.message


def test_jog_stops_when_commands_stop_coming():
    pl = Planner(ROBOT)
    pl.sync(ROBOT.poses["home"])
    pl.jog_joint([1, 0, 0, 0, 0, 0])
    q0 = pl.step(DT)[0]
    for _ in range(40):
        pl.step(DT)
    assert pl.q[0] > q0 and not pl.busy()             # moved, then timed out
    still = pl.q[0]
    pl.step(DT)
    assert pl.q[0] == still


# -------------------------------------------------------------------- safety
def _enabled_safety():
    s = Safety(ROBOT)
    s.on_device_status(True, Mode.IDLE, 0)
    s.sync(ROBOT.poses["home"])
    return s


def test_named_poses_are_inside_the_allowed_workspace():
    s = Safety(ROBOT)
    for name, q in ROBOT.poses.items():
        assert ROBOT.within_limits(q), name
        assert s.check_workspace(q) == "", name


def test_safety_limits_speed_and_acceleration():
    s = _enabled_safety()
    goal = list(ROBOT.poses["home"])
    goal[0] += 1.0                                    # a 57 degree step in one tick
    prev, prev_v = s.filter(goal, DT), 0.0
    for _ in range(200):
        out = s.filter(goal, DT)
        v = (out[0] - prev[0]) / DT
        assert abs(v) <= ROBOT.v_max[0] * 1.01
        assert abs(v - prev_v) / DT <= ROBOT.a_max[0] * 2.1
        prev, prev_v = out, v
    assert abs(prev[0] - goal[0]) < 1e-3


def test_safety_refuses_table_and_base_collisions():
    s = _enabled_safety()
    start = s.filter(None, DT)
    into_table = [0, math.radians(90), math.radians(40), 0, 0, 0]
    assert s.check_workspace(into_table) != ""
    for _ in range(100):
        out = s.filter(into_table, DT)
    assert out == start and "holding" in s.reason     # goal ignored, arm holds


def test_stop_latches_until_cleared():
    s = _enabled_safety()
    goal = list(ROBOT.poses["home"])
    goal[1] += 0.5
    for _ in range(20):
        s.filter(goal, DT)
    s.stop()
    assert s.mode == Mode.STOPPED
    for _ in range(100):
        held = s.filter(goal, DT)
    assert abs(held[1] - goal[1]) > 0.1 and not s.can_move()      # did not continue to the goal
    assert s.filter(goal, DT) == held                             # and stands still
    s.clear()
    assert s.can_move()


def test_safety_blocks_motion_on_device_faults():
    s = _enabled_safety()
    s.on_device_status(True, Mode.IDLE, Fault.BATTERY_CRITICAL)
    assert s.mode == Mode.FAULT and not s.can_move()
    s.on_device_status(False, Mode.DISABLED, 0)
    assert s.mode == Mode.DISABLED


# ----------------------------------------------------------------- simulator
def test_sim_enable_move_and_watchdog():
    arm = SimArm(ROBOT)
    assert arm.enable()
    target = list(ROBOT.poses["home"])
    for _ in range(600):
        arm.touch()
        arm.set_targets(target)
        arm.step(0.01)
    assert max(abs(j.pos - t) for j, t in zip(arm.joints, target)) < 1e-3
    for _ in range(80):                               # PC goes silent for 0.8 s
        arm.step(0.01)
    assert arm.faults & Fault.LINK_LOST
    arm.touch()
    arm.set_targets(ROBOT.poses["rest"])
    arm.step(0.01)
    assert abs(arm.joints[1].target - target[1]) < 0.01   # refused while LINK_LOST is latched
    arm.clear()
    assert not arm.faults & Fault.LINK_LOST


def test_sim_critical_battery_parks_the_arm():
    arm = SimArm(ROBOT, battery_v=6.5)
    arm.mode = Mode.IDLE
    for j, p in zip(arm.joints, ROBOT.poses["home"]):
        j.reset(p)
    for _ in range(1500):
        arm.touch()
        arm.set_targets(ROBOT.poses["home"])          # the PC keeps asking: must be ignored
        arm.step(0.01)
    assert arm.faults & Fault.BATTERY_CRITICAL
    assert max(abs(j.pos - p) for j, p in zip(arm.joints, ROBOT.poses["rest"])) < 1e-3
