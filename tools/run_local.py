"""Run a dataflow WITHOUT dora-rs, in one Python process.

Role in the system
------------------
A development aid, not a replacement for dora. It reads the same dataflow YAML,
creates the same node logic classes and passes messages between them the way
the YAML says. Use it when dora is not installed yet, to debug node logic with
a normal Python debugger, or in tests.

    python tools/run_local.py                      (simulation)
    python tools/run_local.py dora/dataflows/hardware.yml

What it does NOT give you: separate processes, dora's logging and tooling, real
timing guarantees. Learn dora with `dora run` (docs/software/dora_tutorial.md).
"""
from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "dora"))

from exxeta_arm.runtime import Logic  # noqa: E402


def load_logic(path: Path) -> Logic:
    """Import a node's main.py and instantiate the Logic subclass defined in it."""
    spec = importlib.util.spec_from_file_location(f"node_{path.parent.name}", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    for obj in vars(mod).values():
        if isinstance(obj, type) and issubclass(obj, Logic) and obj is not Logic and obj.__module__ == mod.__name__:
            return obj()
    raise RuntimeError(f"no Logic class in {path}")


class LocalDataflow:
    def __init__(self, yml: Path):
        spec = yaml.safe_load(yml.read_text(encoding="utf-8"))
        self.nodes, self.routes, self.timers = {}, {}, []
        for n in spec["nodes"]:
            self.nodes[n["id"]] = load_logic((yml.parent / n["path"]).resolve())
            for input_name, source in (n.get("inputs") or {}).items():
                source = source["source"] if isinstance(source, dict) else source
                if source.startswith("dora/timer/millis/"):
                    self.timers.append([n["id"], input_name, int(source.rsplit("/", 1)[1]) / 1000.0, 0.0])
                else:
                    self.routes.setdefault(source, []).append((n["id"], input_name))

    def deliver(self, node_id, input_name, data):
        """Give one input to a node, then pass on everything it emitted (depth-first)."""
        logic = self.nodes[node_id]
        logic.on_input(input_name, data)
        for output, payload in logic.take_outputs():
            for dst, dst_input in self.routes.get(f"{node_id}/{output}", []):
                self.deliver(dst, dst_input, payload)

    def spin_once(self, now):
        for t in self.timers:
            if now >= t[3]:
                t[3] = now + t[2]
                self.deliver(t[0], t[1], None)

    def run_virtual(self, seconds: float, step: float = 0.002):
        """Advance the dataflow on virtual time (used by tests: 10 simulated seconds take a fraction of a second)."""
        from exxeta_arm import runtime
        if not hasattr(self, "_vt"):
            self._vt = 0.0
            runtime.set_clock(lambda: self._vt)
        end = self._vt + seconds
        while self._vt < end:
            self.spin_once(self._vt)
            self._vt += step

    def run(self):
        try:
            while True:
                self.spin_once(time.monotonic())
                time.sleep(0.002)
        except KeyboardInterrupt:
            pass
        finally:
            for n in self.nodes.values():
                n.close()


if __name__ == "__main__":
    flow = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "dora" / "dataflows" / "sim.yml"
    print(f"local runner (no dora): {flow}")
    LocalDataflow(flow.resolve()).run()
