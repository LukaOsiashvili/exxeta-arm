# Pin map

Board: **ESP32 DevKit, 38 pins, ESP32-WROOM-32** (classic ESP32). This table and `firmware/include/pinmap.h` must always agree; to use another board variant, change both.

| Function | GPIO | Notes |
|---|---|---|
| Servo J1 base yaw | 13 | 50 Hz PWM, 3.3 V logic |
| Servo J2 shoulder | 16 | |
| Servo J3 elbow | 17 | |
| Servo J4 wrist pitch | 18 | |
| Servo J5 wrist roll | 19 | |
| Servo J6 gripper | 23 | |
| Pot J1 (optional) | 36 (VP) | ADC1, input only |
| Pot J2 (optional) | 39 (VN) | ADC1, input only |
| Pot J3 (optional) | 34 | ADC1, input only |
| Pot J4 (optional) | 35 | ADC1, input only |
| Battery voltage | 32 | ADC1, through the 100 kΩ / 33 kΩ divider + 100 nF |
| Spare analog input | 33 | ADC1, free (J5 pot or battery mid-point later, ADR-0002) |
| RGB status LED (WS2812/SK6812) | 27 | data in |
| I2C SDA / SCL (reserved) | 21 / 22 | kept free for an optional ADS1115 |
| UDP port (Wi-Fi) | n/a | 47800 |

## Why these pins

- **Analog inputs only on ADC1** (GPIO 32–39). ADC2 does not work while Wi-Fi is on.
- **No strapping pins** (0, 2, 5, 12, 15) and no flash pins (6–11): a servo or LED on those can stop the board from booting.
- GPIO 34–39 are input-only, which is fine for the pots and useless for servos.
- If a servo misbehaves on 3.3 V logic, add a 74AHCT125 level shifter between the ESP32 and the servo signal pins (not expected to be needed).
