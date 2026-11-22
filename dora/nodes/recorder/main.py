"""Node `recorder`: record & replay (PROJECT_SPEC 8.4).

Inputs : tick (50 Hz), ui_command, estimated_state, planner_status
Outputs: program_command (to the planner), recorder_status

Two kinds of recording, both stored under data/recordings/ :
  * waypoint programs  <name>.json : a list of steps (joint pose, or Cartesian line),
    each with speed, gripper angle and dwell time. Made with "Add waypoint" in the app.
  * continuous recordings <name>.csv : time-stamped joint angles captured at 50 Hz.

Playback never drives the arm directly: every step goes to the planner as a
normal command, so it passes the planner AND the safety node.

Commands: program_save {name, steps} | program_delete {name} | play {name, speed, loop}
          pause | resume | step | play_stop | rec_start {name} | rec_stop
"""
import csv
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from exxeta_arm.robot import PROJECT_ROOT  # noqa: E402
from exxeta_arm.runtime import Logic, clock, run  # noqa: E402

REC_DIR = PROJECT_ROOT / "data" / "recordings"
FORMAT_VERSION = 1


def safe_name(name: str) -> str:
    keep = "".join(c for c in str(name) if c.isalnum() or c in "-_ ").strip().replace(" ", "_")
    return keep[:40] or "untitled"


class Recorder(Logic):
    def __init__(self):
        super().__init__()
        REC_DIR.mkdir(parents=True, exist_ok=True)
        self.q = [0.0] * 6
        self.planner_busy = False
        # continuous recording
        self.rec_file = self.rec_writer = None
        self.rec_name, self.rec_t0, self.rec_rows = "", 0.0, 0
        # playback
        self.play = None            # dict(name, kind, steps/rows, index, speed, loop, paused, single, wait_until, sent, t0)
        self.message = ""

    # ------------------------------------------------------------- storage
    def listing(self):
        programs = sorted(p.stem for p in REC_DIR.glob("*.json"))
        recordings = sorted(p.stem for p in REC_DIR.glob("*.csv"))
        return programs, recordings

    def load_program(self, name):
        return json.loads((REC_DIR / f"{safe_name(name)}.json").read_text(encoding="utf-8"))

    # ------------------------------------------------------------ commands
    def on_command(self, d):
        cmd = d.get("cmd")
        if cmd == "program_save":
            doc = {"format": "exxeta-arm-program", "version": FORMAT_VERSION, "unit": "rad/m/s",
                   "joint_names": ["base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll", "gripper"],
                   "name": safe_name(d["name"]), "steps": d["steps"]}
            (REC_DIR / f"{doc['name']}.json").write_text(json.dumps(doc, indent=2), encoding="utf-8")
            self.message = f"saved program '{doc['name']}'"
        elif cmd == "program_load":
            try:
                self.emit("recorder_status", self.status(program=self.load_program(d["name"])))
            except Exception as e:
                self.message = f"cannot load: {e}"
        elif cmd == "program_delete":
            for ext in (".json", ".csv"):
                (REC_DIR / f"{safe_name(d['name'])}{ext}").unlink(missing_ok=True)
        elif cmd == "rec_start" and not self.rec_file:
            self.rec_name = safe_name(d.get("name") or time.strftime("rec_%Y%m%d_%H%M%S"))
            self.rec_file = open(REC_DIR / f"{self.rec_name}.csv", "w", newline="", encoding="utf-8")
            self.rec_file.write(f"# exxeta-arm-recording version {FORMAT_VERSION}, joint angles in rad, 50 Hz\n")
            self.rec_writer = csv.writer(self.rec_file)
            self.rec_writer.writerow(["t", "base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll", "gripper"])
            self.rec_t0, self.rec_rows = clock(), 0
        elif cmd == "rec_stop" and self.rec_file:
            self.rec_file.close()
            self.rec_file = self.rec_writer = None
            self.message = f"recorded '{self.rec_name}' ({self.rec_rows} samples)"
        elif cmd == "play":
            self.start_play(d)
        elif cmd in ("play_stop", "stop", "disable"):
            if self.play:
                self.message = "playback stopped"
            self.play = None
        elif self.play and cmd == "pause":
            self.play["paused"] = True
        elif self.play and cmd == "resume":
            self.play["paused"], self.play["single"] = False, False
        elif self.play and cmd == "step":                 # step-through: run exactly one step, then pause again
            self.play["paused"], self.play["single"] = False, True

    def start_play(self, d):
        name = safe_name(d["name"])
        base = {"name": name, "index": 0, "speed": float(d.get("speed", 1.0)), "loop": bool(d.get("loop")),
                "paused": False, "single": False, "wait_until": 0.0, "sent": False, "t0": None}
        try:
            if (REC_DIR / f"{name}.json").exists():
                self.play = base | {"kind": "program", "steps": self.load_program(name)["steps"]}
            else:
                with open(REC_DIR / f"{name}.csv", encoding="utf-8") as f:
                    rows = [r for r in csv.reader(l for l in f if not l.startswith("#"))][1:]
                self.play = base | {"kind": "recording", "rows": [[float(v) for v in r] for r in rows]}
            self.message = f"playing '{name}'"
        except Exception as e:
            self.message = f"cannot play '{name}': {e}"

    # ------------------------------------------------------------ playback
    def tick_play(self):
        p = self.play
        if not p or p["paused"]:
            return
        now = clock()
        if p["kind"] == "recording":                      # stream the recorded angles, time-scaled
            if not p["rows"]:
                self.play = None
                return
            if p["t0"] is None:
                p["t0"] = now
                self.emit("program_command", {"cmd": "move_joints", "q": p["rows"][0][1:7]})   # go to the start first
                p["sent"] = True
                return
            if p["sent"]:
                if self.planner_busy or now - p["t0"] < 0.3:
                    return
                p["sent"], p["t0"] = False, now
            t = (now - p["t0"]) * p["speed"] + p["rows"][0][0]
            while p["index"] + 1 < len(p["rows"]) and p["rows"][p["index"] + 1][0] <= t:
                p["index"] += 1
            self.emit("program_command", {"cmd": "follow", "q": p["rows"][p["index"]][1:7]})
            if p["index"] + 1 >= len(p["rows"]):
                self.finish_or_loop()
            return
        # waypoint program: send a step, wait until the planner is done, dwell, next
        if p["index"] >= len(p["steps"]):
            self.finish_or_loop()
            return
        step = p["steps"][p["index"]]
        if not p["sent"]:
            speed = min(1.0, float(step.get("speed", 0.5)) * p["speed"])
            if step.get("type") == "cartesian":
                self.emit("program_command", {"cmd": "move_cartesian", "pose": step["pose"], "speed": speed})
            else:
                q = list(step["q"][:5]) + [step.get("gripper", step["q"][5] if len(step["q"]) > 5 else 0.0)]
                self.emit("program_command", {"cmd": "move_joints", "q": q, "speed": speed})
            p["sent"], p["t0"] = True, now
        elif now - p["t0"] > 0.15 and not self.planner_busy:
            if not p["wait_until"]:
                p["wait_until"] = now + float(step.get("dwell", 0.0))
            if now >= p["wait_until"]:
                p["index"] += 1
                p["sent"], p["wait_until"] = False, 0.0
                if p["single"]:
                    p["paused"] = True

    def finish_or_loop(self):
        p = self.play
        if p["loop"]:
            p["index"], p["sent"], p["t0"], p["wait_until"] = 0, False, None, 0.0
        else:
            self.message = f"finished '{p['name']}'"
            self.play = None

    # ---------------------------------------------------------------- loop
    def status(self, program=None):
        programs, recordings = self.listing()
        p = self.play
        st = {"programs": programs, "recordings": recordings, "recording": bool(self.rec_file),
              "recording_name": self.rec_name if self.rec_file else "", "samples": self.rec_rows,
              "playing": p["name"] if p else "", "paused": bool(p and p["paused"]), "index": p["index"] if p else 0,
              "total": (len(p["steps"]) if p["kind"] == "program" else len(p["rows"])) if p else 0,
              "message": self.message}
        if program is not None:
            st["program"] = program
        return st

    def on_input(self, name, data):
        if name == "estimated_state" and isinstance(data, dict):
            self.q = data["q"]
        elif name == "planner_status" and isinstance(data, dict):
            self.planner_busy = data["busy"]
        elif name == "ui_command" and isinstance(data, dict):
            self.on_command(data)
        elif name == "tick":
            if self.rec_writer:
                self.rec_writer.writerow([f"{clock() - self.rec_t0:.3f}"] + [f"{v:.5f}" for v in self.q])
                self.rec_rows += 1
            self.tick_play()
            self.emit("recorder_status", self.status())

    def close(self):
        if self.rec_file:
            self.rec_file.close()


if __name__ == "__main__":
    run(Recorder())
