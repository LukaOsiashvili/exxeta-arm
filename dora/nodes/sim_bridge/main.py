"""Node `sim_bridge`: a simulated arm. Drop-in replacement for `arm_bridge`.

Inputs : tick (50 Hz timer), joint_targets (6 x rad), control ({"cmd": ...})
Outputs: joint_state, device_status, battery     (identical to arm_bridge)

With this node the complete stack, web app included, runs on a laptop with no
hardware (dataflows/sim.yml). It is the default way to develop.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from exxeta_arm.protocol import Mode, fault_names  # noqa: E402
from exxeta_arm.robot import load_robot  # noqa: E402
from exxeta_arm.runtime import Logic, Stopwatch, load_config, run  # noqa: E402
from exxeta_arm.sim import SimArm  # noqa: E402


class SimBridge(Logic):
    def __init__(self, config=None):
        super().__init__()
        cfg = (config or load_config()).get("sim", {})
        self.robot = load_robot()
        self.arm = SimArm(self.robot, with_pots=cfg.get("pots", False), battery_v=cfg.get("battery_v", 8.1))
        self.connected = cfg.get("auto_connect", True)
        self.clock = Stopwatch(0.02)

    def on_input(self, name, data):
        if name == "joint_targets" and self.connected:
            self.arm.touch()
            self.arm.set_targets(data)
        elif name == "control" and isinstance(data, dict):
            cmd = data.get("cmd")
            if cmd == "connect":
                self.connected = True
            elif cmd == "disconnect":
                self.arm.disable()
                self.connected = False
            elif self.connected:
                self.arm.touch()
                {"enable": self.arm.enable, "disable": self.arm.disable, "stop": self.arm.stop,
                 "clear": self.arm.clear}.get(cmd, lambda: None)()
        elif name == "tick":
            dt = self.clock.dt()
            if self.connected:
                self.arm.touch()                       # the bridge itself is the heartbeat
            for _ in range(2):                         # firmware runs at 100 Hz
                self.arm.step(dt / 2)
            self.publish()

    def publish(self):
        s = self.arm.state()
        rad = math.radians
        self.emit("joint_state", {"target": [rad(a) for a in s.target_deg], "position": [rad(a) for a in s.position_deg],
                                  "pot": [rad(a) for a in s.pot_deg], "pot_valid": s.pot_valid})
        self.emit("device_status", {"connected": self.connected, "simulated": True, "transport": "sim",
                                    "mode": s.mode if self.connected else Mode.DISABLED,
                                    "faults": s.faults, "fault_names": fault_names(s.faults),
                                    "name": "simulated-arm", "firmware": "sim", "latency_ms": 0, "link_quality": 100,
                                    "ports": [], "error": ""})
        self.emit("battery", {"voltage": round(s.battery_v, 2), "percent": s.battery_pct,
                              "charging": False})


if __name__ == "__main__":
    run(SimBridge())
