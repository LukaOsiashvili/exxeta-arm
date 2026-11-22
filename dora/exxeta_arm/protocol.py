"""exxeta-arm wire protocol, version 1 (PC side).

Role in the system
------------------
The one protocol spoken between the PC (`arm_bridge` node) and the ESP32, over
USB serial and over Wi-Fi/UDP. The C++ twin of this file is
firmware/lib/armcore/protocol.{h,cpp}. Both are tested against the same file of
example frames (tests/protocol_vectors.json), so they cannot drift apart.
Full description: docs/software/protocol.md.

Frame layout (all integers little-endian):

    ver:u8 = 1 | type:u8 | seq:u16 | payload ... | crc:u16

    crc = CRC-16/CCITT-FALSE over everything before it.

Transport framing:
  * UDP    : one frame per datagram, exactly as above.
  * serial : 0x00, COBS(frame), 0x00.  COBS removes all zero bytes from the frame,
             so a zero byte always marks a frame boundary and the firmware can
             tell binary frames from the human-readable debug CLI.

Angles travel as centi-degrees (int16), speeds as deci-degrees per second (uint16).
Only the standard library is used.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

VERSION = 1
N_JOINTS = 6
N_POTS = 4            # J1..J4 can have pots (ADR-0002)


class Msg:
    # PC -> device
    HELLO = 0x01
    HEARTBEAT = 0x02
    ENABLE = 0x03
    DISABLE = 0x04
    SET_JOINT_TARGETS = 0x05
    STOP = 0x06
    CLEAR = 0x07          # acknowledge a latched STOP / LINK_LOST
    RELEASE = 0x08        # give up control so another PC can take over
    GET_CONFIG = 0x10
    SET_CONFIG = 0x11
    CALIBRATION = 0x12
    # device -> PC
    ACK = 0x80
    HELLO_ACK = 0x81
    STATE = 0x82
    LOG = 0x83
    CONFIG = 0x90


class Mode:
    DISABLED, IDLE, MOVING, STOPPED, FAULT = range(5)
    NAMES = ["DISABLED", "IDLE", "MOVING", "STOPPED", "FAULT"]


class Fault:
    LINK_LOST = 1 << 0
    BATTERY_WARN = 1 << 1
    BATTERY_CRITICAL = 1 << 2
    SERVO_POWER_OFF = 1 << 3
    STOP_LATCHED = 1 << 4
    LIMIT_CLAMPED = 1 << 5
    CHARGER_PRESENT = 1 << 6
    NAMES = {1: "LINK_LOST", 2: "BATTERY_WARN", 4: "BATTERY_CRITICAL", 8: "SERVO_POWER_OFF",
             16: "STOP_LATCHED", 32: "LIMIT_CLAMPED", 64: "CHARGER_PRESENT"}


class Status:
    OK, BUSY, BAD_TOKEN, BAD_ARG, REFUSED = range(5)
    NAMES = ["OK", "BUSY", "BAD_TOKEN", "BAD_ARG", "REFUSED"]


class Cfg:
    SERVO_CAL = 1       # index = joint: min_us:u16 max_us:u16 zero_us:u16 us_per_deg_x100:i16 (sign = direction)
    JOINT_LIMITS = 2    # index = joint: min_cdeg:i16 max_cdeg:i16 max_speed_ddeg_s:u16 max_accel_deg_s2:u16
    POT_CAL = 3         # index = joint (0..3): adc_at_zero:u16 adc_per_deg_x100:i16 enabled:u8
    DEVICE_NAME = 4     # utf-8 text
    AUTH_TOKEN = 5      # u32
    WIFI = 6            # index = slot 0..2: ssid_len:u8 ssid pass_len:u8 pass   (write only)
    WATCHDOG_MS = 7     # u16
    STATE_RATE_HZ = 8   # u8
    REST_POSE = 9       # 6 x i16 centi-degrees


class Cal:
    JOG_PULSE = 1       # joint:u8 pulse_us:u16  : drive one servo with a raw pulse width (calibration only)
    SAVE = 2            # write the current configuration to flash (NVS)
    FACTORY_RESET = 3


# --------------------------------------------------------------------------
# CRC and COBS
# --------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (polynomial 0x1021, start value 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Consistent Overhead Byte Stuffing: the result contains no zero byte."""
    out = bytearray()
    block = bytearray()
    for b in data:
        if b == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
        else:
            block.append(b)
            if len(block) == 254:
                out.append(255)
                out += block
                block.clear()
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError("bad COBS data")
        out += data[i + 1:i + code]
        i += code
        if code < 255 and i < len(data):
            out.append(0)
    return bytes(out)


# --------------------------------------------------------------------------
# Frames
# --------------------------------------------------------------------------
@dataclass
class Frame:
    type: int
    seq: int = 0
    payload: bytes = b""


def encode_frame(f: Frame) -> bytes:
    body = struct.pack("<BBH", VERSION, f.type, f.seq & 0xFFFF) + f.payload
    return body + struct.pack("<H", crc16(body))


def decode_frame(raw: bytes) -> Frame:
    if len(raw) < 6:
        raise ValueError("frame too short")
    body, (crc,) = raw[:-2], struct.unpack("<H", raw[-2:])
    if crc16(body) != crc:
        raise ValueError("bad CRC")
    ver, typ, seq = struct.unpack("<BBH", body[:4])
    if ver != VERSION:
        raise ValueError(f"protocol version {ver}, expected {VERSION}")
    return Frame(typ, seq, body[4:])


def to_serial(f: Frame) -> bytes:
    return b"\x00" + cobs_encode(encode_frame(f)) + b"\x00"


class SerialDeframer:
    """Feed it raw serial bytes; it returns complete frames and collects CLI/log text lines."""

    def __init__(self):
        self._buf = bytearray()
        self._in_frame = False
        self.text_lines: list[str] = []
        self.errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        frames = []
        for b in data:
            if b == 0:
                if self._in_frame and self._buf:
                    try:
                        frames.append(decode_frame(cobs_decode(bytes(self._buf))))
                    except ValueError:
                        self.errors += 1
                    self._in_frame = False
                else:
                    self._in_frame = True
                self._buf.clear()
            elif self._in_frame:
                self._buf.append(b)
            elif b == 0x0A:
                self.text_lines.append(self._buf.decode("utf-8", "replace").rstrip("\r"))
                self._buf.clear()
            else:
                self._buf.append(b)
            if len(self._buf) > 512:
                self._buf.clear()
                self._in_frame = False
                self.errors += 1
        return frames


# --------------------------------------------------------------------------
# Payload builders (PC -> device)
# --------------------------------------------------------------------------
def _cdeg(deg: float) -> int:
    return max(-32768, min(32767, int(round(deg * 100))))


def hello(token: int, client_id: int) -> bytes:
    return struct.pack("<II", token & 0xFFFFFFFF, client_id & 0xFFFFFFFF)


def enable(use_pots: bool = False) -> bytes:
    return struct.pack("<B", 1 if use_pots else 0)


def set_joint_targets(targets_deg, max_speed_deg_s=None) -> bytes:
    """6 targets in degrees; optional 6 speed caps in deg/s (0 or None = firmware default)."""
    speeds = max_speed_deg_s or [0] * N_JOINTS
    return (struct.pack("<6h", *[_cdeg(a) for a in targets_deg])
            + struct.pack("<6H", *[max(0, min(65535, int(round(s * 10)))) for s in speeds]))


def get_config(key: int, index: int = 0) -> bytes:
    return struct.pack("<BB", key, index)


def set_config(key: int, index: int, data: bytes) -> bytes:
    return struct.pack("<BB", key, index) + data


def servo_cal(min_us: int, max_us: int, zero_us: int, us_per_deg: float) -> bytes:
    return struct.pack("<HHHh", min_us, max_us, zero_us, int(round(us_per_deg * 100)))


def joint_limits(min_deg: float, max_deg: float, max_speed_deg_s: float, max_accel_deg_s2: float) -> bytes:
    return struct.pack("<hhHH", _cdeg(min_deg), _cdeg(max_deg), int(round(max_speed_deg_s * 10)), int(round(max_accel_deg_s2)))


def pot_cal(adc_at_zero: int, adc_per_deg: float, enabled: bool) -> bytes:
    return struct.pack("<HhB", adc_at_zero, int(round(adc_per_deg * 100)), 1 if enabled else 0)


def wifi(ssid: str, password: str) -> bytes:
    s, p = ssid.encode(), password.encode()
    return bytes([len(s)]) + s + bytes([len(p)]) + p


def cal_jog(joint: int, pulse_us: int) -> bytes:
    return struct.pack("<BBH", Cal.JOG_PULSE, joint, pulse_us)


# --------------------------------------------------------------------------
# Payload parsers (device -> PC)
# --------------------------------------------------------------------------
@dataclass
class HelloAck:
    status: int
    proto: int
    fw: tuple
    n_joints: int
    name: str


def parse_hello_ack(p: bytes) -> HelloAck:
    status, proto, a, b, c, n, ln = struct.unpack("<7B", p[:7])
    return HelloAck(status, proto, (a, b, c), n, p[7:7 + ln].decode("utf-8", "replace"))


def parse_ack(p: bytes) -> tuple[int, int, int]:
    """-> (acknowledged seq, acknowledged type, status)"""
    return struct.unpack("<HBB", p[:4])


@dataclass
class State:
    mode: int = 0
    faults: int = 0
    target_deg: list = field(default_factory=lambda: [0.0] * N_JOINTS)      # last commanded targets
    position_deg: list = field(default_factory=lambda: [0.0] * N_JOINTS)    # where the firmware is driving the servos now
    pot_deg: list = field(default_factory=lambda: [0.0] * N_POTS)
    pot_valid: int = 0                                                       # bit i = pot of joint i is fitted and plausible
    battery_v: float = 0.0
    battery_pct: int = 0
    uptime_ms: int = 0
    rx_ok: int = 0
    rx_bad: int = 0
    last_seq: int = 0


_STATE_FMT = "<BH6h6h4hBHBIHHH"
STATE_SIZE = struct.calcsize(_STATE_FMT)


def pack_state(s: State) -> bytes:
    return struct.pack(_STATE_FMT, s.mode, s.faults, *[_cdeg(a) for a in s.target_deg], *[_cdeg(a) for a in s.position_deg],
                       *[_cdeg(a) for a in s.pot_deg], s.pot_valid, int(round(s.battery_v * 1000)), s.battery_pct,
                       s.uptime_ms & 0xFFFFFFFF, s.rx_ok & 0xFFFF, s.rx_bad & 0xFFFF, s.last_seq & 0xFFFF)


def parse_state(p: bytes) -> State:
    v = struct.unpack(_STATE_FMT, p[:STATE_SIZE])
    return State(mode=v[0], faults=v[1], target_deg=[x / 100 for x in v[2:8]], position_deg=[x / 100 for x in v[8:14]],
                 pot_deg=[x / 100 for x in v[14:18]], pot_valid=v[18], battery_v=v[19] / 1000, battery_pct=v[20],
                 uptime_ms=v[21], rx_ok=v[22], rx_bad=v[23], last_seq=v[24])


def fault_names(faults: int) -> list[str]:
    return [n for bit, n in Fault.NAMES.items() if faults & bit]
