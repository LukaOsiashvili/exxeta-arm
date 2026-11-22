"""Connection to the real arm: USB serial or Wi-Fi/UDP, same frames on both.

Role in the system
------------------
Used by the `arm_bridge` node. `ArmLink` opens one of the two transports,
performs the HELLO handshake (token + single-controller check), and then just
moves frames. It never blocks: `poll()` returns whatever has arrived.
Switching transport at runtime = close() and connect() with other settings.
"""
from __future__ import annotations

import random
import socket
import time

from . import protocol as pr

UDP_PORT = 47800


def list_serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports
        return [p.device for p in list_ports.comports()]
    except Exception:
        return []


class ArmLink:
    def __init__(self):
        self.transport = None           # "serial" | "wifi" | None
        self._ser = None
        self._sock = None
        self._addr = None
        self._deframer = pr.SerialDeframer()
        self._seq = 0
        self.client_id = random.getrandbits(32)
        self.info: pr.HelloAck | None = None
        self.error = ""
        self.text_lines: list[str] = []
        self.rx_frames = 0
        self.last_rx = 0.0

    # ------------------------------------------------------------ lifecycle
    def connect(self, transport: str, port: str = "", host: str = "", token: int = 0, timeout: float = 2.0) -> bool:
        self.close()
        self.error = ""
        try:
            if transport == "serial":
                import serial
                self._ser = serial.Serial(port, 115200, timeout=0, write_timeout=0.2)
                time.sleep(0.1)
                self._ser.reset_input_buffer()
            elif transport == "wifi":
                self._addr = (socket.gethostbyname(host), UDP_PORT)   # also resolves exxeta-arm.local where mDNS works
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._sock.setblocking(False)
            else:
                raise ValueError(f"unknown transport '{transport}'")
            self.transport = transport
        except Exception as e:                       # port busy, host not found, ...
            self.error = str(e)
            self.close()
            return False
        # handshake: repeat HELLO until the arm answers
        deadline = time.monotonic() + timeout
        next_hello = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_hello:
                self.send(pr.Msg.HELLO, pr.hello(token, self.client_id))
                next_hello = time.monotonic() + 0.3
            for f in self.poll():
                if f.type == pr.Msg.HELLO_ACK:
                    ack = pr.parse_hello_ack(f.payload)
                    if ack.status == pr.Status.OK:
                        self.info = ack
                        return True
                    self.error = {pr.Status.BUSY: "arm is controlled by another PC (BUSY)",
                                  pr.Status.BAD_TOKEN: "wrong auth token"}.get(ack.status, "handshake refused")
                    self.close()
                    return False
            time.sleep(0.01)
        self.error = "no answer from the arm (check cable/port, IP address and that it is powered)"
        self.close()
        return False

    def close(self):
        if self.connected:
            try:
                self.send(pr.Msg.RELEASE)
            except Exception:
                pass
        for obj in (self._ser, self._sock):
            try:
                if obj:
                    obj.close()
            except Exception:
                pass
        self._ser = self._sock = None
        self.transport, self.info = None, None

    @property
    def connected(self) -> bool:
        return self.info is not None

    # ------------------------------------------------------------------- io
    def send(self, msg_type: int, payload: bytes = b"") -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        f = pr.Frame(msg_type, self._seq, payload)
        try:
            if self._ser:
                self._ser.write(pr.to_serial(f))
            elif self._sock:
                self._sock.sendto(pr.encode_frame(f), self._addr)
        except Exception as e:
            self.error = str(e)
        return self._seq

    def poll(self) -> list[pr.Frame]:
        frames = []
        try:
            if self._ser:
                data = self._ser.read(4096)
                if data:
                    frames = self._deframer.feed(data)
                    self.text_lines += self._deframer.text_lines
                    self._deframer.text_lines = []
            elif self._sock:
                while True:
                    try:
                        data, _ = self._sock.recvfrom(1024)
                    except (BlockingIOError, ConnectionResetError):
                        break
                    try:
                        frames.append(pr.decode_frame(data))
                    except ValueError:
                        pass
        except Exception as e:                       # cable pulled, ...
            self.error = str(e)
            self.close()
        if frames:
            self.rx_frames += len(frames)
            self.last_rx = time.monotonic()
        return frames
