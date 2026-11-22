"""Node `safety`: the last gate on the PC before the bridge.

Inputs : tick (50 Hz), joint_goals (from the planner), ui_command, device_status,
         estimated_state, planner_status, battery
Outputs: joint_targets (6 x rad, always a safe pose), control (to the bridge), safety_status

It owns the commands that change the state of the arm: connect, disconnect,
enable, disable, stop, clear. STOP is latched here AND sent to the firmware.
Everything else it forwards is filtered by exxeta_arm/safety.py (limits, speed,
keep-out zones). See PROJECT_SPEC 10.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from exxeta_arm.protocol import Mode  # noqa: E402
from exxeta_arm.robot import load_robot  # noqa: E402
from exxeta_arm.runtime import Logic, Stopwatch, load_config, run  # noqa: E402
from exxeta_arm.safety import Safety  # noqa: E402

FORWARD = ("connect", "disconnect", "jog_pulse", "set_servo_cal", "set_pot_cal", "set_wifi", "save_config")


class SafetyNode(Logic):
    def __init__(self, config=None):
        super().__init__()
        cfg = (config or load_config()).get("safety", {})
        self.robot = load_robot()
        self.safety = Safety(self.robot, speed_scale=cfg.get("speed_scale", 1.0))
        self.goal = None
        self.q_est = list(self.robot.poses["rest"])
        self.device_mode = Mode.DISABLED
        self.battery = {}
        self.clock = Stopwatch(0.02)

    def on_command(self, d):
        cmd, s = d.get("cmd"), self.safety
        if cmd == "stop":                              # software STOP: decelerate and hold, latched
            s.stop()
            self.goal = None
            self.emit("control", {"cmd": "stop"})
        elif cmd == "clear":                           # acknowledge the STOP / LINK_LOST
            s.clear()
            s.sync(self.q_est)
            self.goal = None
            self.emit("control", {"cmd": "clear"})
        elif cmd == "enable":
            if not s.stop_latched:
                self.goal = None
                self.emit("control", {"cmd": "enable"})
        elif cmd == "disable":
            self.goal = None
            self.emit("control", {"cmd": "disable"})
        elif cmd in FORWARD:
            if cmd in ("jog_pulse",) and s.mode != Mode.DISABLED:
                return                                 # raw servo pulses only while the arm is disabled
            self.emit("control", d)

    def on_input(self, name, data):
        s = self.safety
        if name == "joint_goals":
            self.goal = data
        elif name == "estimated_state" and isinstance(data, dict):
            self.q_est = data["q_cmd"]
        elif name == "battery" and isinstance(data, dict):
            self.battery = data
        elif name == "device_status" and isinstance(data, dict):
            prev = self.device_mode
            self.device_mode = data["mode"]
            if prev == Mode.DISABLED and self.device_mode != Mode.DISABLED:
                s.sync(self.q_est)                     # just enabled: start from the arm's position, no jump
                self.goal = None
            s.on_device_status(data["connected"], data["mode"], data["faults"])
        elif name == "planner_status" and isinstance(data, dict):
            if data.get("park_done"):                  # "Park & power off": arm is in the rest pose
                self.emit("control", {"cmd": "disable"})
        elif name == "ui_command" and isinstance(data, dict):
            self.on_command(data)
        elif name == "tick":
            dt = self.clock.dt()
            if s.mode != Mode.DISABLED:
                self.emit("joint_targets", s.filter(self.goal, dt))
            self.emit("safety_status", {"mode": s.mode, "mode_name": Mode.NAMES[s.mode], "stop_latched": s.stop_latched,
                                        "reason": s.reason, "can_move": s.can_move(),
                                        "q_safe": [m.pos for m in s.motion],
                                        "limits": {"min": self.robot.q_min, "max": self.robot.q_max}})


if __name__ == "__main__":
    run(SafetyNode())
