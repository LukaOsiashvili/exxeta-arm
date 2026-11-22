"""Smallest possible PC client for the arm: no dora, one servo, one script.

Role: bench bring-up (docs/firmware/bench_test.md, step 3) and a readable example
of the protocol. It connects, enables, wiggles one joint and prints the state.

    python tools/bench_client.py COM5              (USB serial)
    python tools/bench_client.py 192.168.1.50      (Wi-Fi: IP address or exxeta-arm.local)
    python tools/bench_client.py COM5 --joint 2 --delta 15

Stop with Ctrl+C. If you kill it instead (close the window), the arm's watchdog
must take over: it holds position and blinks red.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dora"))

from exxeta_arm import protocol as pr  # noqa: E402
from exxeta_arm.device import ArmLink  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("target", help="serial port (COM5, /dev/ttyUSB0) or host name / IP address")
    ap.add_argument("--joint", type=int, default=1, help="joint to move, 1-6")
    ap.add_argument("--delta", type=float, default=20.0, help="how far to move it, degrees")
    ap.add_argument("--token", type=int, default=0)
    args = ap.parse_args()

    link = ArmLink()
    is_net = "." in args.target and not args.target.startswith("/")
    if not link.connect("wifi" if is_net else "serial", port=args.target, host=args.target, token=args.token):
        sys.exit(f"could not connect: {link.error}")
    print(f"connected to '{link.info.name}', firmware {link.info.fw}")

    state = pr.State()
    link.send(pr.Msg.CLEAR)
    link.send(pr.Msg.ENABLE, pr.enable())
    base, t0, last_print = None, time.monotonic(), 0.0
    try:
        while True:
            for f in link.poll():
                if f.type == pr.Msg.STATE:
                    state = pr.parse_state(f.payload)
                elif f.type == pr.Msg.ACK and pr.parse_ack(f.payload)[2] != pr.Status.OK:
                    print("arm refused a command:", pr.Status.NAMES[pr.parse_ack(f.payload)[2]])
            if state.mode in (pr.Mode.IDLE, pr.Mode.MOVING):
                if base is None:
                    base = list(state.position_deg)        # start from where the arm says it is
                target = list(base)
                phase = int((time.monotonic() - t0) / 3) % 4   # +delta, back, -delta, back; 3 s each
                target[args.joint - 1] += (args.delta, 0, -args.delta, 0)[phase]
                link.send(pr.Msg.SET_JOINT_TARGETS, pr.set_joint_targets(target))
            else:
                link.send(pr.Msg.HEARTBEAT)
            if time.monotonic() - last_print > 0.5:
                last_print = time.monotonic()
                print(f"mode {pr.Mode.NAMES[state.mode]:8s} faults {pr.fault_names(state.faults)} "
                      f"J{args.joint} {state.position_deg[args.joint - 1]:7.2f} deg  battery {state.battery_v:.2f} V")
            time.sleep(0.02)                               # 50 Hz, also the heartbeat
    except KeyboardInterrupt:
        link.send(pr.Msg.DISABLE)
        link.close()
        print("disabled and released")


if __name__ == "__main__":
    main()
