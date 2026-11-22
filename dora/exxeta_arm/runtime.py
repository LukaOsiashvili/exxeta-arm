"""Glue between node logic and dora-rs.

Role in the system
------------------
Every node is written as a small class with one method, `on_input(name, data)`,
that calls `self.emit(output_name, data)`. That class knows nothing about dora.
`run(logic)` connects it to the dora dataflow: it receives events, decodes the
Arrow data into plain Python, calls the class and sends what it emitted.

Why this split: the same classes are driven by tools/run_local.py (no dora
needed) and by the unit tests, and the dora-specific code lives in one place
(dora API verified against dora-rs 1.1.0: `Node()`, `for event in node`,
`node.send_output(id, pyarrow.Array, metadata)`).

Message convention (see docs/software/dora_tutorial.md):
  * a list of numbers  -> Arrow float64 array            (joint_goals, joint_targets)
  * anything else      -> one JSON string in an Arrow string array
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

from .robot import PROJECT_ROOT

CONFIG_PATH = Path(os.environ.get("EXXETA_ARM_CONFIG", PROJECT_ROOT / "config" / "arm.yaml"))


def load_config() -> dict:
    """config/arm.yaml as a dict (all tunable values live there, not in the code)."""
    import yaml
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class Logic:
    """Base class of all node logic classes."""

    def __init__(self):
        self._outbox = []

    def emit(self, output: str, data):
        self._outbox.append((output, data))

    def take_outputs(self):
        out, self._outbox = self._outbox, []
        return out

    def on_input(self, name: str, data):      # override
        pass

    def close(self):                          # override if the node owns threads, ports or files
        pass


def encode(data):
    import pyarrow as pa
    if isinstance(data, (list, tuple)) and data and all(isinstance(v, (int, float)) for v in data):
        return pa.array([float(v) for v in data], type=pa.float64())
    return pa.array([json.dumps(data)])


def decode(value):
    import pyarrow as pa
    if value is None or len(value) == 0:
        return None
    if pa.types.is_string(value.type) or pa.types.is_large_string(value.type):
        return json.loads(value[0].as_py())
    return value.to_pylist()


def run(logic: Logic):
    """Run a node inside a dora dataflow until the dataflow stops."""
    from dora import Node
    node = Node()
    try:
        for event in node:
            if event is None:
                break
            kind = event["type"]
            if kind == "INPUT":
                logic.on_input(event["id"], decode(event["value"]))
                for output, data in logic.take_outputs():
                    node.send_output(output, encode(data))
            elif kind == "STOP":
                break
            elif kind == "ERROR":
                print(f"[{type(logic).__name__}] dora error: {event.get('error')}", file=sys.stderr)
    finally:
        logic.close()


_clock = time.monotonic


def clock() -> float:
    """Monotonic time in seconds. Node logic uses this instead of time.monotonic() so that
    tests and tools/run_local.py can run the whole dataflow on virtual (faster than real) time."""
    return _clock()


def set_clock(fn):
    global _clock
    _clock = fn


class Stopwatch:
    """Measures the real time between ticks (timers are not perfectly regular)."""

    def __init__(self, nominal: float):
        self.nominal, self._last = nominal, None

    def dt(self) -> float:
        now = clock()
        dt = self.nominal if self._last is None else min(max(now - self._last, 0.001), 0.1)
        self._last = now
        return dt
