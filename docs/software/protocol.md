# PC ↔ arm protocol, version 1

One protocol, two transports. Implementations: `dora/exxeta_arm/protocol.py` (PC) and `firmware/lib/armcore/protocol.{h,cpp}` (ESP32). Both are tested against the same example frames in `tests/protocol_vectors.json`.

## Frame

All integers are little-endian.

| Field | Size | Meaning |
|---|---|---|
| `ver` | 1 | protocol version, always 1 |
| `type` | 1 | message type (table below) |
| `seq` | 2 | sequence number chosen by the sender |
| `payload` | 0–190 | depends on the type |
| `crc` | 2 | CRC-16/CCITT-FALSE (poly 0x1021, start 0xFFFF) over everything before it |

## Transports

- **Wi-Fi:** UDP port **47800**, one frame per datagram. The arm answers to the address the frame came from.
- **USB serial:** 115200 baud. Each frame is sent as `0x00`, COBS(frame), `0x00`. COBS removes every zero byte from the frame, so a zero always marks a frame boundary. Anything outside frames that ends in a newline is a line of the debug CLI (try `help`).

## Session rules

1. The PC sends `HELLO` with the auth token and a random client id. The arm answers `HELLO_ACK`.
2. **One controller at a time.** A `HELLO` from another client gets status `BUSY` until the first releases control (`RELEASE`) or is silent for 3 s.
3. Every valid frame from the controller feeds the **watchdog**. After 500 ms of silence (configurable) the arm holds its position and latches `LINK_LOST`. It refuses new targets until `CLEAR`.
4. Streamed messages (`SET_JOINT_TARGETS`, `STATE`) are not acknowledged: the newest one wins. Commands and configuration get an `ACK` with a status.

## Messages PC → arm

| Type | Name | Payload | Reply |
|---|---|---|---|
| 0x01 | HELLO | token:u32, client_id:u32 | HELLO_ACK |
| 0x02 | HEARTBEAT | none | none |
| 0x03 | ENABLE | flags:u8 (bit 0: start from pot readings) | ACK |
| 0x04 | DISABLE | none | ACK |
| 0x05 | SET_JOINT_TARGETS | 6 × target:i16 (0.01°), then 6 × max speed:u16 (0.1 °/s, 0 = default) | none |
| 0x06 | STOP | none (decelerate, hold, latch) | ACK |
| 0x07 | CLEAR | none (acknowledge STOP / LINK_LOST) | ACK |
| 0x08 | RELEASE | none | ACK |
| 0x10 | GET_CONFIG | key:u8, index:u8 | CONFIG (same seq) |
| 0x11 | SET_CONFIG | key:u8, index:u8, data | ACK |
| 0x12 | CALIBRATION | sub:u8 … | ACK |

Calibration sub-commands: `1` JOG_PULSE (joint:u8, pulse_us:u16: raw pulse on one servo), `2` SAVE (write configuration to flash), `3` FACTORY_RESET.

## Messages arm → PC

| Type | Name | Payload |
|---|---|---|
| 0x80 | ACK | acked seq:u16, acked type:u8, status:u8 |
| 0x81 | HELLO_ACK | status:u8, protocol:u8, firmware version 3 × u8, joints:u8, name length:u8, name |
| 0x82 | STATE | see below, sent 50 times a second |
| 0x83 | LOG | level:u8, text |
| 0x90 | CONFIG | key:u8, index:u8, data |

Status codes: 0 OK, 1 BUSY, 2 BAD_TOKEN, 3 BAD_ARG, 4 REFUSED.

### STATE (49 bytes)

| Field | Type | Meaning |
|---|---|---|
| mode | u8 | 0 DISABLED, 1 IDLE, 2 MOVING, 3 STOPPED, 4 FAULT |
| faults | u16 | bit set, see below |
| target | 6 × i16 | last commanded targets, 0.01° |
| position | 6 × i16 | where the firmware is driving the servos right now, 0.01° |
| pot | 4 × i16 | pot readings J1–J4, 0.01° |
| pot_valid | u8 | bit i set = pot of joint i fitted and plausible |
| battery | u16 | pack voltage, mV |
| battery_pct | u8 | 0–100 |
| uptime | u32 | ms |
| rx_ok, rx_bad | u16, u16 | frame counters |
| last_seq | u16 | seq of the last frame received (the PC measures latency with it) |

Fault bits: 1 LINK_LOST, 2 BATTERY_WARN, 4 BATTERY_CRITICAL, 8 SERVO_POWER_OFF, 16 STOP_LATCHED, 32 LIMIT_CLAMPED, 64 CHARGER_PRESENT.

## Configuration keys

| Key | Name | Index | Data |
|---|---|---|---|
| 1 | SERVO_CAL | joint 0–5 | min_us:u16, max_us:u16, zero_us:u16, us_per_deg × 100:i16 (sign = direction) |
| 2 | JOINT_LIMITS | joint 0–5 | min:i16, max:i16 (0.01°), max speed:u16 (0.1 °/s), max accel:u16 (°/s²) |
| 3 | POT_CAL | joint 0–3 | adc_at_zero:u16, adc_per_deg × 100:i16, enabled:u8 |
| 4 | DEVICE_NAME | 0 | UTF-8 text |
| 5 | AUTH_TOKEN | 0 | u32 (write only; 0 = no token) |
| 6 | WIFI | slot 0–2 | ssid length:u8, ssid, password length:u8, password (write only) |
| 7 | WATCHDOG_MS | 0 | u16 |
| 8 | STATE_RATE_HZ | 0 | u8 |
| 9 | REST_POSE | 0 | 6 × i16 (0.01°) |

Changes take effect at once but are lost at power-off unless `CALIBRATION / SAVE` follows.

## Changing the protocol

1. Change `protocol.py` and `protocol.h/.cpp` together. Raise `VERSION` if old and new cannot talk to each other.
2. Run `python tools/gen_firmware_defaults.py` to regenerate the shared test vectors.
3. Run `python -m pytest tests` and, on a PC with a C++ compiler, `pio test -e native`.
