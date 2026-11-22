# Learning exercises

All exercises run in simulation (`dora run dataflows/sim.yml` or `python tools/run_local.py`). Read the [dora tutorial](../software/dora_tutorial.md) first. Hints are below each task; reference solutions are in [solutions.md](solutions.md), kept apart on purpose. The solutions are sketches written by the implementer and have **not** been run.

To let a new node move the arm, feed its output into the planner's `program_command` input. A dora input has one source, so for the exercises replace the recorder's connection in your copy of the dataflow:

```yaml
      program_command: my_node/program_command
```

## 1. Draw a square (dora basics, kinematics)

Write a node that makes the grip centre trace a 6 cm square in the Y–Z plane, 18 cm in front of the base, and repeats.

- Hint: send `{"cmd": "move_cartesian", "pose": [x, y, z, pitch, roll]}` and wait until `planner_status["busy"]` is false before sending the next corner.
- Hint: start from the `ready` pose so all four corners are reachable. Keep the pitch the arm has there.
- Check: watch the X read-out in the app; it should stay within a millimetre.

## 2. Pick and place (waypoint programs)

Using only the web app, build a program that picks an imaginary object at the front left and puts it down at the front right: approach from above, descend, close, lift, move, descend, open, lift.

- Hint: PROGRAMS tab, jog to each pose, + WAYPOINT. The gripper angle is stored with each waypoint.
- Hint: add a dwell of 0.5 s after closing and opening.
- Then open the saved JSON in `data/recordings/` and change one waypoint by hand.

## 3. A new safety rule (the safety node)

Add a rule: the hand may not go higher than 30 cm above the table.

- Hint: `check_workspace()` in `dora/exxeta_arm/safety.py` returns a reason string or `""`.
- Add a test to `tests/test_core.py` that proves the rule: a pose above the limit is refused, the `zero` pose is now refused, `home` still passes.
- Question: the built-in `zero` pose now fails `test_named_poses_are_inside_the_allowed_workspace`. What should happen to it?

## 4. A simulated obstacle (state and messages)

Write a node that publishes a box obstacle on the table, and extend the safety node so the hand refuses to enter it.

- Hint: publish `{"min": [x, y, z], "max": [x, y, z]}` once per second; add it as an input to `safety`.
- Hint: the safety check works in the arm's side view (r, z). For a box you need x and y too: use `kinematics.fk()` on the candidate pose.
- Stretch: draw the box in the 3D view.

## 5. Gamepad control (a node with its own thread)

Control the arm with a USB gamepad: left stick X/Y, right stick Z and pitch, triggers for the gripper.

- Hint: `pygame.joystick` reads gamepads on all three operating systems. Reading blocks, so use a thread and a queue, as `web_server` does.
- Hint: send `{"cmd": "jog_cart", "vel": [vx, vy, vz, vpitch, vroll, vgrip]}` ten times a second with values from −1 to 1. The planner stops by itself 0.3 s after the last command: release the stick and the arm stops.
- Add a dead zone, or the arm creeps.

## 6. Camera node starter (preparing for vision)

Add a node that grabs frames from a webcam and publishes them, and a second node that prints the frame size and rate.

- Hint: `cv2.VideoCapture(0)`; publish `frame.ravel()` as a `pyarrow` uint8 array and the shape in the metadata: `node.send_output("image", pa.array(frame.ravel()), {"width": w, "height": h})`.
- This needs the real dora API (bytes and metadata), not the project's `emit()` helper: a good moment to write a node without `runtime.py`.
- Question: at 30 frames a second, what does the default input queue (10 messages, oldest dropped) do for a slow consumer?

## 7. Firmware: a seventh status (C++)

Make the LED show white for two seconds after a successful `ENABLE`.

- Hint: `ledUpdate()` in `firmware/src/main.cpp`; remember when enable happened with `millis()`.
- Build with `pio run`. Why must this not use `delay()`?

## 8. Change the robot (the single source of truth)

Make the forearm 10 mm longer.

- Change `forearm_length_mm` in `cad/params/arm_params.json`, run `python tools/export_all.py`.
- What happens to the shoulder's safety factor? Does the sizing gate still pass? What changed in the URDF, the 3D view and `python -m pytest tests`?
- Put the value back afterwards.
