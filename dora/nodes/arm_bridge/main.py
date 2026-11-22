"""Node `arm_bridge`: talks to the ESP32 over USB serial or Wi-Fi (switchable at runtime).

Inputs : tick (50 Hz timer), joint_targets (6 x rad), control ({"cmd": ...})
Outputs: joint_state, device_status, battery

Control commands: connect {transport, port, host} | disconnect | enable | disable |
stop | clear | jog_pulse {joint, us} | set_servo_cal {joint, ...} | set_pot_cal |
save_config | set_wifi {ssid, password}.

It sends the newest joint targets every tick (that stream is also the heartbeat
the firmware watchdog needs) and turns STATE frames into dataflow messages.
After connecting it uploads config/calibration/<device>.json if that file exists.
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from exxeta_arm import protocol as pr  # noqa: E402
from exxeta_arm.device import ArmLink, list_serial_ports  # noqa: E402
from exxeta_arm.robot import PROJECT_ROOT  # noqa: E402
from exxeta_arm.runtime import Logic, load_config, run  # noqa: E402

CAL_DIR = PROJECT_ROOT / "config" / "calibration"


class ArmBridge(Logic):
    def __init__(self, config=None):
        super().__init__()
        self.cfg = (config or load_config()).get("arm", {})
        self.link = ArmLink()
        self.state = pr.State()
        self.targets = None
        self.ports, self._ports_at = [], 0.0
        self._ping_sent, self.latency_ms = {}, 0.0
        self._rx_window, self._window_start, self.quality = 0, time.monotonic(), 0
        self.last_error = ""
        if self.cfg.get("auto_connect", False):
            self.connect(self.cfg)

    # -------------------------------------------------------------- commands
    def connect(self, d):
        ok = self.link.connect(d.get("transport", self.cfg.get("transport", "serial")),
                               port=d.get("port", self.cfg.get("serial_port", "")),
                               host=d.get("host", self.cfg.get("wifi_host", "exxeta-arm.local")),
                               token=int(self.cfg.get("token", 0)))
        self.last_error = "" if ok else self.link.error
        self.targets = None
        if ok:
            self.upload_calibration()

    def upload_calibration(self):
        f = CAL_DIR / f"{self.link.info.name}.json"
        if not f.exists():
            return
        cal = json.loads(f.read_text(encoding="utf-8"))
        for i, c in enumerate(cal.get("servos", [])):
            self.link.send(pr.Msg.SET_CONFIG, pr.set_config(pr.Cfg.SERVO_CAL, i, pr.servo_cal(
                c["min_us"], c["max_us"], c["zero_us"], c["us_per_deg"])))
        for i, c in enumerate(cal.get("pots", [])):
            self.link.send(pr.Msg.SET_CONFIG, pr.set_config(pr.Cfg.POT_CAL, i, pr.pot_cal(
                c["adc_at_zero"], c["adc_per_deg"], c["enabled"])))

    def on_control(self, d):
        cmd = d.get("cmd")
        if cmd == "connect":
            self.connect(d)
        elif cmd == "disconnect":
            self.link.close()
        elif not self.link.connected:
            return
        elif cmd == "enable":
            self.targets = None
            self.link.send(pr.Msg.ENABLE, pr.enable(bool(self.state.pot_valid)))
        elif cmd == "disable":
            self.link.send(pr.Msg.DISABLE)
        elif cmd == "stop":
            self.link.send(pr.Msg.STOP)
        elif cmd == "clear":
            self.targets = None
            self.link.send(pr.Msg.CLEAR)
        elif cmd == "jog_pulse":
            self.link.send(pr.Msg.CALIBRATION, pr.cal_jog(int(d["joint"]), int(d["us"])))
        elif cmd == "set_servo_cal":
            self.remember_cal("servos", int(d["joint"]), 6, {k: d[k] for k in ("min_us", "max_us", "zero_us", "us_per_deg")})
            self.link.send(pr.Msg.SET_CONFIG, pr.set_config(pr.Cfg.SERVO_CAL, int(d["joint"]), pr.servo_cal(
                int(d["min_us"]), int(d["max_us"]), int(d["zero_us"]), float(d["us_per_deg"]))))
        elif cmd == "set_pot_cal":
            self.remember_cal("pots", int(d["joint"]), 4, {k: d[k] for k in ("adc_at_zero", "adc_per_deg", "enabled")})
            self.link.send(pr.Msg.SET_CONFIG, pr.set_config(pr.Cfg.POT_CAL, int(d["joint"]), pr.pot_cal(
                int(d["adc_at_zero"]), float(d["adc_per_deg"]), bool(d["enabled"]))))
        elif cmd == "set_wifi":
            self.link.send(pr.Msg.SET_CONFIG, pr.set_config(pr.Cfg.WIFI, 0, pr.wifi(d["ssid"], d["password"])))
        elif cmd == "save_config":
            self.link.send(pr.Msg.CALIBRATION, bytes([pr.Cal.SAVE]))

    def remember_cal(self, kind, index, count, values):
        """Keep a copy of the calibration on the PC (config/calibration/<device>.json, PROJECT_SPEC 7.4)."""
        f = CAL_DIR / f"{self.link.info.name}.json"
        cal = json.loads(f.read_text(encoding="utf-8")) if f.exists() else {"version": 1}
        default = ({"min_us": 500, "max_us": 2500, "zero_us": 1500, "us_per_deg": 11.11} if kind == "servos"
                   else {"adc_at_zero": 2048, "adc_per_deg": 13.65, "enabled": False})
        items = cal.setdefault(kind, [dict(default) for _ in range(count)])
        items[index] = values
        CAL_DIR.mkdir(parents=True, exist_ok=True)
        f.write_text(json.dumps(cal, indent=2), encoding="utf-8")

    # ----------------------------------------------------------------- loop
    def on_input(self, name, data):
        if name == "joint_targets":
            self.targets = data
        elif name == "control" and isinstance(data, dict):
            self.on_control(data)
        elif name == "tick":
            self.tick()

    def tick(self):
        now = time.monotonic()
        if self.link.connected:
            # the target stream doubles as heartbeat; HEARTBEAT is sent when there is nothing to stream
            if self.targets is not None and self.state.mode in (pr.Mode.IDLE, pr.Mode.MOVING):
                seq = self.link.send(pr.Msg.SET_JOINT_TARGETS, pr.set_joint_targets([math.degrees(a) for a in self.targets]))
            else:
                seq = self.link.send(pr.Msg.HEARTBEAT)
            self._ping_sent[seq] = now
            for f in self.link.poll():
                if f.type == pr.Msg.STATE:
                    self.state = pr.parse_state(f.payload)
                    sent = self._ping_sent.pop(self.state.last_seq, None)
                    if sent:
                        self.latency_ms += 0.2 * ((now - sent) * 1000 - self.latency_ms)
                    self._rx_window += 1
                elif f.type == pr.Msg.ACK:
                    _, typ, status = pr.parse_ack(f.payload)
                    if status != pr.Status.OK:
                        self.last_error = f"arm refused command 0x{typ:02X}: {pr.Status.NAMES[status]}"
                elif f.type == pr.Msg.LOG:
                    print("[arm]", f.payload[1:].decode("utf-8", "replace"))
            if len(self._ping_sent) > 200:
                self._ping_sent.clear()
            if not self.link.connected:                       # poll() closed the link (cable pulled)
                self.last_error = self.link.error
            elif now - self.link.last_rx > 2.0 and self.link.last_rx:
                self.last_error = "connection lost (no data from the arm)"
                self.link.close()
        if now - self._window_start >= 1.0:                   # link quality = share of expected STATE frames
            self.quality = min(100, int(self._rx_window * 2)) if self.link.connected else 0
            self._rx_window, self._window_start = 0, now
        if now - self._ports_at > 2.0 and not self.link.connected:
            self.ports, self._ports_at = list_serial_ports(), now
        self.publish()

    def publish(self):
        s, rad, c = self.state, math.radians, self.link.connected
        self.emit("joint_state", {"target": [rad(a) for a in s.target_deg], "position": [rad(a) for a in s.position_deg],
                                  "pot": [rad(a) for a in s.pot_deg], "pot_valid": s.pot_valid if c else 0})
        info = self.link.info
        self.emit("device_status", {
            "connected": c, "simulated": False, "transport": self.link.transport or "",
            "mode": s.mode if c else pr.Mode.DISABLED, "faults": s.faults if c else 0,
            "fault_names": pr.fault_names(s.faults) if c else [],
            "name": info.name if info else "", "firmware": ".".join(map(str, info.fw)) if info else "",
            "latency_ms": round(self.latency_ms, 1), "link_quality": self.quality,
            "ports": self.ports, "error": self.last_error})
        self.emit("battery", {"voltage": round(s.battery_v, 2) if c else 0.0, "percent": s.battery_pct if c else 0,
                              "charging": bool(s.faults & pr.Fault.CHARGER_PRESENT) if c else False})

    def close(self):
        self.link.close()


if __name__ == "__main__":
    run(ArmBridge())
