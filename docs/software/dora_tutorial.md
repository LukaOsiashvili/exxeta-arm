# dora-rs tutorial, using this project

dora-rs is a framework for building a robot's software as a **dataflow**: small programs (nodes) that send each other messages. It plays the role ROS 2 plays elsewhere, with far less to install. This page explains it with the files in this repository.

Versions used: dora-rs 1.1.0 and dora CLI 1.1.0, Python 3.11 or newer. The implementer could not run dora on the build machine (see `docs/decisions/unverified.md`); the nodes were verified with `tools/run_local.py` instead.

## 1. The four ideas

1. **Node:** one program with one job. Ours are in `dora/nodes/<name>/main.py`.
2. **Output and input:** a node publishes named outputs; another node subscribes by naming `node_id/output_name` as one of its inputs.
3. **Dataflow:** a YAML file listing the nodes and their connections. dora starts every node as its own process and carries the messages.
4. **Arrow:** messages are Apache Arrow arrays, a compact, typed, language-independent memory format. dora passes them between processes without copying.

## 2. Our dataflow

`dora/dataflows/sim.yml`:

```
web_server ──ui_command──▶ planner ──joint_goals──▶ safety ──joint_targets──▶ bridge
     ▲                        ▲                       │  └──control──────────▶   │
     │                        └──── safety_status ────┘                          │
     │                                                                joint_state│
     └── estimated_state, ee_pose, battery, … ◀── state_estimator ◀──────────────┘
                                    kinematics ◀──┘        recorder ──program_command──▶ planner
```

| Node | Job |
|---|---|
| `bridge` | the arm: `sim_bridge` (simulated) or `arm_bridge` (ESP32 over USB or Wi-Fi) |
| `state_estimator` | best estimate of the joint angles (pots where fitted) |
| `kinematics` | live position of the hand; answers "is this point reachable?" |
| `planner` | turns commands into a smooth stream of joint goals |
| `safety` | last gate: limits, speed, keep-out zones, STOP |
| `recorder` | record and replay |
| `web_server` | the web app and its WebSocket |

`hardware.yml` is the same file with one line changed: the `bridge` node's `path`. That is the point of the design: everything else cannot tell simulation from hardware.

## 3. A node in dora's own terms

This is all the dora API the project uses (it lives in `dora/exxeta_arm/runtime.py`):

```python
from dora import Node
import pyarrow as pa

node = Node()                                  # identity comes from the dataflow
for event in node:                             # blocks until something arrives
    if event["type"] == "INPUT":
        name = event["id"]                     # which input
        values = event["value"].to_pylist()    # Arrow array -> Python list
        node.send_output("doubled", pa.array([v * 2 for v in values]))
    elif event["type"] == "STOP":
        break
```

Things to remember:

- `send_output` takes a **pyarrow array** (or bytes), never a plain list.
- A timer is just an input: `tick: dora/timer/millis/20` delivers an event every 20 ms.
- Each input has a queue (default 10 messages, oldest dropped). A slow node therefore sees fresh data rather than a growing backlog.

## 4. How nodes are written in this project

To keep node code testable without dora, each node is a class with one method:

```python
from exxeta_arm.runtime import Logic, run

class Doubler(Logic):
    def on_input(self, name, data):
        if name == "numbers":
            self.emit("doubled", [v * 2 for v in data])

if __name__ == "__main__":
    run(Doubler())            # connects the class to dora
```

`run()` does the loop from section 3. `emit()` converts for you:

- a list of numbers becomes an Arrow `float64` array,
- anything else (a dict, usually) becomes one JSON string in an Arrow string array.

Look at `dora/nodes/state_estimator/main.py`: the whole node is 15 lines.

## 5. Run it

```bash
cd dora
dora run dataflows/sim.yml
```

`dora run` starts everything in the foreground and stops it with Ctrl+C. Useful extras:

```bash
dora graph dataflows/sim.yml          # draw the dataflow
dora up                               # start the background coordinator and daemon
dora start dataflows/sim.yml --name sim
dora list
dora logs sim planner                 # what one node printed
dora stop sim
dora down
```

`dora run … --uv` creates the Python environments for you if you use `uv`.

Without dora installed, `python tools/run_local.py` runs the same nodes in one process. It is handy with a debugger, but it is not dora: one thread, no separate processes.

## 6. Add your own node: a worked example

Goal: a node that prints a line whenever the hand goes below 5 cm.

1. Create `dora/nodes/low_watch/main.py`:

```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from exxeta_arm.runtime import Logic, run

class LowWatch(Logic):
    def __init__(self):
        super().__init__()
        self.was_low = False

    def on_input(self, name, data):
        if name == "ee_pose":
            low = data["z"] < 0.05
            if low and not self.was_low:
                print(f"hand is low: z = {data['z'] * 1000:.0f} mm")
                self.emit("alert", {"z": data["z"]})
            self.was_low = low

if __name__ == "__main__":
    run(LowWatch())
```

2. Add it to `dataflows/sim.yml`:

```yaml
  - id: low_watch
    path: ../nodes/low_watch/main.py
    inputs:
      ee_pose: kinematics/ee_pose
    outputs: [alert]
```

3. Run the dataflow and drive the hand down.

That is the whole pattern. The [exercises](../learning/exercises.md) build on it.

## 7. Rules of this project

- **Units are SI** in every message: radians, metres, seconds. Degrees and millimetres appear only in the web app and in the wire protocol to the ESP32.
- **Only `safety` commands the bridge.** A new node that wants to move the arm sends commands to the planner (as `program_command`, like the recorder does) and so automatically passes the planner and the safety node.
- **No tunable numbers in code:** `config/arm.yaml` for runtime settings, `cad/params/arm_params.json` for the robot itself.
- Never do slow work (file access in a loop, network calls, `time.sleep`) inside `on_input`: the node cannot receive anything meanwhile. The web server shows the pattern for slow work: its own thread and a queue.
