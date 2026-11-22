"""Node `kinematics`: forward kinematics of the live arm and IK queries.

Inputs : estimated_state, ik_query {"id", "pose": [x, y, z, pitch, roll], "relax": bool}
Outputs: ee_pose {"x","y","z","pitch","roll"}  (grip centre, metres / radians)
         ik_result {"id", "ok", "q", "reason", "pitch_relaxed"}

The web app uses ik_query while the user drags the 3D target, to show at once
whether a point is reachable. Trajectories are computed inside the planner with
the same library (exxeta_arm/kinematics.py), because a straight line needs
hundreds of IK calls with continuity checks (ADR-0008).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from exxeta_arm import kinematics as kin  # noqa: E402
from exxeta_arm.robot import load_robot  # noqa: E402
from exxeta_arm.runtime import Logic, run  # noqa: E402


class Kinematics(Logic):
    def __init__(self):
        super().__init__()
        self.robot = load_robot()
        self.q = list(self.robot.poses["rest"])

    def on_input(self, name, data):
        if name == "estimated_state" and isinstance(data, dict):
            self.q = data["q"]
            p = kin.fk(self.robot, self.q)
            self.emit("ee_pose", {"x": p.x, "y": p.y, "z": p.z, "pitch": p.pitch, "roll": p.roll})
        elif name == "ik_query" and isinstance(data, dict):
            res = kin.ik(self.robot, kin.Pose(*data["pose"]), self.q, allow_pitch_relax=bool(data.get("relax")))
            self.emit("ik_result", {"id": data.get("id"), "ok": res.ok, "q": res.q, "reason": res.reason,
                                    "pitch_relaxed": res.pitch_relaxed})


if __name__ == "__main__":
    run(Kinematics())
