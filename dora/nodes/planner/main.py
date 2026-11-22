"""Node `planner`: turns commands into a smooth 50 Hz stream of joint goals.

Inputs : tick (50 Hz), ui_command, program_command (from the recorder),
         estimated_state, safety_status
Outputs: joint_goals (6 x rad), planner_status

Commands (from the web app or the recorder), all {"cmd": ...}:
  move_joints {q, speed?} | move_pose {name} | move_cartesian {pose:[x,y,z,pitch,roll]}
  jog_joint {vel:[6]} | jog_cart {vel:[6]} | gripper {angle} | follow {q}
  set_speed {scale} | park | pose_save {name} | pose_delete {name} | pose_rename {name, to}

The planner only produces goals while the safety node reports IDLE or MOVING.
Each time the arm becomes enabled (or a STOP is cleared) it re-reads the arm's
actual position first, so motion always starts from where the arm really is.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from exxeta_arm import kinematics as kin  # noqa: E402
from exxeta_arm.planner import Planner  # noqa: E402
from exxeta_arm.protocol import Mode  # noqa: E402
from exxeta_arm.robot import PROJECT_ROOT, load_robot  # noqa: E402
from exxeta_arm.runtime import Logic, Stopwatch, load_config, run  # noqa: E402

POSES_FILE = PROJECT_ROOT / "config" / "poses.json"
BUILT_IN = ("rest", "home", "ready", "zero")


class PlannerNode(Logic):
    def __init__(self, config=None):
        super().__init__()
        cfg = (config or load_config()).get("planner", {})
        self.robot = load_robot()
        self.planner = Planner(self.robot, speed_scale=cfg.get("speed_scale", 0.4))
        self.max_speed_scale = cfg.get("max_speed_scale", 1.0)
        self.q_est = list(self.robot.poses["rest"])
        self.safety_mode = Mode.DISABLED
        self.parking = False
        self.clock = Stopwatch(0.02)
        self.load_poses()

    # ------------------------------------------------------------ user poses
    def load_poses(self):
        if POSES_FILE.exists():
            for name, q in json.loads(POSES_FILE.read_text(encoding="utf-8")).get("poses", {}).items():
                if name not in BUILT_IN:
                    self.robot.poses[name] = q

    def save_poses(self):
        POSES_FILE.parent.mkdir(parents=True, exist_ok=True)
        user = {k: v for k, v in self.robot.poses.items() if k not in BUILT_IN}
        POSES_FILE.write_text(json.dumps({"version": 1, "unit": "rad", "poses": user}, indent=2), encoding="utf-8")

    # -------------------------------------------------------------- commands
    def on_command(self, d):
        pl, cmd = self.planner, d.get("cmd")
        active = self.safety_mode in (Mode.IDLE, Mode.MOVING) and pl.synced
        if cmd == "set_speed":
            pl.speed_scale = min(max(float(d["scale"]), 0.05), self.max_speed_scale)
        elif cmd == "pose_save":
            if d["name"] in BUILT_IN:
                pl.message = "built-in poses cannot be overwritten"
            else:
                self.robot.poses[d["name"]] = list(self.q_est)
                self.save_poses()
        elif cmd == "pose_delete" and d.get("name") not in BUILT_IN:
            self.robot.poses.pop(d.get("name"), None)
            self.save_poses()
        elif cmd == "pose_rename" and d.get("name") not in BUILT_IN and d.get("name") in self.robot.poses:
            self.robot.poses[d["to"]] = self.robot.poses.pop(d["name"])
            self.save_poses()
        elif cmd in ("stop", "disable"):
            pl.cancel()
            self.parking = False
        elif not active:
            if cmd in ("move_joints", "move_pose", "move_cartesian", "jog_joint", "jog_cart", "gripper", "park"):
                pl.message = "arm is not enabled"
        elif cmd == "move_joints":
            pl.move_joints(d["q"], d.get("speed"))
        elif cmd == "move_pose":
            pl.move_pose(d["name"], d.get("speed"))
        elif cmd == "move_cartesian":
            pl.move_cartesian(kin.Pose(*d["pose"]), d.get("speed"))
        elif cmd == "jog_joint":
            pl.jog_joint(d["vel"])
        elif cmd == "jog_cart":
            pl.jog_cartesian(d["vel"])
        elif cmd == "gripper":
            pl.set_gripper(float(d["angle"]), d.get("speed"))
        elif cmd == "follow":                      # replay of a continuous recording: the safety node smooths it
            pl.cancel()
            pl.q = self.robot.clamp(d["q"])
        elif cmd == "park":                        # "Park & power off": go to the rest pose, then ask for disable
            self.parking = pl.move_pose("rest")

    # ------------------------------------------------------------------ loop
    def on_input(self, name, data):
        if name == "estimated_state" and isinstance(data, dict):
            self.q_est = data["q"]
        elif name == "safety_status" and isinstance(data, dict):
            mode = data["mode"]
            was_active = self.safety_mode in (Mode.IDLE, Mode.MOVING)
            self.safety_mode = mode
            if mode in (Mode.IDLE, Mode.MOVING) and not was_active:
                self.planner.sync(data.get("q_safe") or self.q_est)     # start from where the arm really is
            elif mode not in (Mode.IDLE, Mode.MOVING):
                self.planner.cancel()
                self.planner.synced = False
                self.parking = False
        elif name in ("ui_command", "program_command") and isinstance(data, dict):
            self.on_command(data)
        elif name == "tick":
            dt = self.clock.dt()
            pl = self.planner
            park_done = False
            if self.safety_mode in (Mode.IDLE, Mode.MOVING) and pl.synced:
                self.emit("joint_goals", pl.step(dt))
                if self.parking and not pl.busy():
                    self.parking, park_done = False, True
            self.emit("planner_status", {"status": pl.status, "busy": pl.busy(), "message": pl.message,
                                         "speed_scale": pl.speed_scale, "park_done": park_done,
                                         "poses": {k: v for k, v in self.robot.poses.items()},
                                         "built_in": list(BUILT_IN)})


if __name__ == "__main__":
    run(PlannerNode())
