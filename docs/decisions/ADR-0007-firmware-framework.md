# ADR-0007: Firmware framework and structure

- **Status:** accepted (implementer choice)
- **Spec:** D8 (PlatformIO; Arduino-ESP32 or ESP-IDF, justified), §7

## Decision

**Arduino-ESP32** on PlatformIO, pinned to `espressif32@6.9.0` (Arduino core 2.0.17). No third-party libraries: servo PWM uses the LEDC timers directly (50 Hz, 16 bit), the status LED uses the core's `neopixelWrite`, and Wi-Fi, UDP, mDNS, the setup web page, NVS and OTA all come with the core.

## Why Arduino and not ESP-IDF

- The team has mixed skill levels. Arduino code reads like the tutorials everyone has seen, and `setup()` / `loop()` is easier to follow than FreeRTOS tasks.
- Everything this firmware needs is in the Arduino core. ESP-IDF would add configuration effort without adding a capability we use.
- The firmware is deliberately simple (a "smart I/O" device), so the finer control ESP-IDF offers is not needed. One cooperative loop with a 100 Hz control tick is enough.

## Structure

- `lib/armcore/` holds the **pure logic** (protocol encode/decode, COBS, CRC, motion profile). It has no Arduino includes, so it compiles on a PC for the unit tests (`pio test -e native`).
- `src/main.cpp` holds everything that touches hardware, in clearly marked sections.
- `include/pinmap.h` is the only place pins are named; `include/arm_defaults.h` is generated from `arm_params.json`.

## Details worth knowing

- **Serial framing:** binary frames are `0x00, COBS(frame), 0x00`. A text line that does not start with `0x00` is treated as a debug CLI command. COBS output never contains a zero byte, so the two cannot be confused.
- **Enum names are prefixed** (`MSG_`, `MODE_`, `FLT_`, `ST_`, `CFG_`, `CAL_`) because Arduino defines macros such as `DISABLED` and `OK`.
- **Motion rule:** the braking speed uses the exact formula for discrete time steps. The textbook `sqrt(2·a·distance)` brakes one step late and overshoots; the test suite caught this.
- **OTA** is enabled (ArduinoOTA) but only handled while the arm is disabled.

## Not verified here

No host C++ compiler on the implementer's machine, so the native unit tests were written but not run. The ESP32 build itself succeeds (66 % flash, 16 % RAM).
