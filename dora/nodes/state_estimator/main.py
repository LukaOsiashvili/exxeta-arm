"""Node `state_estimator`: the best available estimate of where each joint is.

Input : joint_state (from the bridge)
Output: estimated_state {"q": [6 x rad], "source": [...], "q_cmd": [...], "q_measured": [...]}

Hobby servos give no feedback. Without pots the estimate is the position the
firmware is currently commanding ("estimated"). Where a pot is fitted and valid,
its reading is used instead ("measured"). Every joint is labelled so the UI can
show the difference (ghost overlay).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from exxeta_arm.runtime import Logic, run  # noqa: E402


class StateEstimator(Logic):
    def on_input(self, name, data):
        if name != "joint_state" or not isinstance(data, dict):
            return
        cmd = data["position"]
        q, source, measured = list(cmd), ["estimated"] * 6, [None] * 6
        for i in range(min(4, len(data.get("pot", [])))):
            if data.get("pot_valid", 0) & (1 << i):
                q[i] = measured[i] = data["pot"][i]
                source[i] = "measured"
        self.emit("estimated_state", {"q": q, "source": source, "q_cmd": cmd, "q_target": data["target"],
                                      "q_measured": measured})


if __name__ == "__main__":
    run(StateEstimator())
