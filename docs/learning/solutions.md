# Reference solutions

Sketches for the [exercises](exercises.md). **They were written without being run**; treat them as a guide, not as tested code. Try the exercise first.

## 1. Draw a square

```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from exxeta_arm.runtime import Logic, run

class Square(Logic):
    def __init__(self):
        super().__init__()
        self.pose = None          # live pose of the hand
        self.corners = None
        self.i = 0
        self.busy = True
        self.wait = 0

    def on_input(self, name, data):
        if name == "ee_pose":
            self.pose = data
        elif name == "planner_status":
            self.busy = data["busy"]
        elif name == "tick" and self.pose:
            if self.corners is None:                      # fix the square once, around a known-good spot
                p, s = self.pose["pitch"], 0.03
                self.corners = [[0.18, y, z, p, 0.0] for y, z in ((-s, 0.15 - s), (s, 0.15 - s), (s, 0.15 + s), (-s, 0.15 + s))]
            self.wait -= 1
            if not self.busy and self.wait <= 0:
                self.emit("program_command", {"cmd": "move_cartesian", "pose": self.corners[self.i]})
                self.i = (self.i + 1) % 4
                self.wait = 10                            # give the planner a few ticks to report busy

if __name__ == "__main__":
    run(Square())
```

Dataflow entry: inputs `tick: dora/timer/millis/50`, `ee_pose: kinematics/ee_pose`, `planner_status: planner/planner_status`; output `program_command`. Enable the arm and move it to `ready` from the app first.

## 2. Pick and place

No code. A typical program has eight waypoints; the saved file looks like this (angles in radians):

```json
{ "format": "exxeta-arm-program", "version": 1, "name": "pick_place",
  "steps": [ { "type": "joint", "q": [0.5, 0.6, 1.2, 0.9, 0, 2.6], "gripper": 2.6, "speed": 0.5, "dwell": 0 } ] }
```

## 3. A new safety rule

In `check_workspace`, inside the loop over the hand samples:

```python
            if pz > 0.30:
                return "hand would go above the height limit"
```

The `zero` pose points 36 cm up, so it now fails. Sensible answers: remove `zero` from the built-in poses, or keep it for calibration only and exempt it explicitly. Changing the test to ignore the failure is the wrong answer: the test found a real conflict between two requirements.

## 4. A simulated obstacle

Obstacle node: on every tick `self.emit("obstacle", {"min": [0.12, -0.05, 0.0], "max": [0.20, 0.05, 0.08]})`.

In `SafetyNode.on_input`: `elif name == "obstacle": self.safety.obstacle = data`.

In `Safety.check_workspace`, after computing the hand samples, convert each sample to x, y with the yaw `q[0]`:

```python
        box = getattr(self, "obstacle", None)
        if box:
            x, y = pr * math.cos(q[0]), pr * math.sin(q[0])
            if all(lo - HAND_RADIUS_M <= v <= hi + HAND_RADIUS_M for v, lo, hi in zip((x, y, pz), box["min"], box["max"])):
                return "hand would hit the obstacle"
```

## 5. Gamepad control

```python
import queue, threading
import pygame

class Gamepad(Logic):
    def __init__(self):
        super().__init__()
        self.q = queue.Queue(maxsize=1)
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):                       # own thread: pygame must not block the dora loop
        pygame.init(); pygame.joystick.init()
        pad = pygame.joystick.Joystick(0)
        clock = pygame.time.Clock()
        dz = lambda v: 0.0 if abs(v) < 0.15 else v
        while True:
            pygame.event.pump()
            vel = [-dz(pad.get_axis(1)), -dz(pad.get_axis(0)), -dz(pad.get_axis(3)), dz(pad.get_axis(2)), 0.0,
                   (pad.get_axis(5) - pad.get_axis(4)) / 2]
            try: self.q.put_nowait(vel)
            except queue.Full: pass
            clock.tick(20)

    def on_input(self, name, data):
        if name == "tick":
            try: vel = self.q.get_nowait()
            except queue.Empty: return
            if any(vel):
                self.emit("program_command", {"cmd": "jog_cart", "vel": vel})
```

Axis numbers differ between gamepads: print them first.

## 6. Camera node starter

```python
import cv2, pyarrow as pa
from dora import Node

node, cam = Node(), cv2.VideoCapture(0)
for event in node:
    if event["type"] == "INPUT" and event["id"] == "tick":
        ok, frame = cam.read()
        if ok:
            h, w, _ = frame.shape
            node.send_output("image", pa.array(frame.ravel()), {"width": w, "height": h, "encoding": "bgr8"})
    elif event["type"] == "STOP":
        break
```

The queue: a slow consumer simply misses frames and always works on recent ones. For a camera that is what you want; for commands that must not be lost it is not.

## 7. A seventh status

Add `static uint32_t enabledAtMs = 0;`, set it to `millis()` in `cmdEnable` where the mode becomes idle, and at the top of the colour choice in `ledUpdate()`:

```cpp
  if (enabledAtMs && t - enabledAtMs < 2000) { r = g = b = 20; }
  else if (apMode) { ... }
```

`delay()` would stop the whole loop: no motion updates, no watchdog, no serial for two seconds.

## 8. Change the robot

Expected: the shoulder safety factor drops from about 2.24 to about 2.1 (longer lever for everything beyond the elbow), still above 2.0, so the gate passes. The URDF's `wrist_pitch` joint origin moves by 10 mm, the 3D view shows the longer forearm after a reload, and the kinematics tests still pass because they read the same parameter. `test_fk_horizontal_reach` fails: it has the number 0.255 written into it, which is a reminder that tests with hard-coded geometry must follow the design.
