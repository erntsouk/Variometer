# NX-VARIO

A home-built paragliding variometer on an Arduino Nano, based on the original NX-VARIO by [XONIX2004](https://www.thingiverse.com/XONIX2004). It beeps when you climb, plays a continuous deep tone when you sink, and has a soft "buzzer" that helps you find weak thermals.

![Wiring](docs/wiring.png)

## Features

- **Fast vario:** the barometer (BMP180) and accelerometer (MPU6050) are combined, so the vario reacts in about 0.05 s.
- **Climb sound:** beeps get faster and higher as the lift gets stronger.
- **Sink sound:** a continuous tone that gets deeper as you sink faster.
- **Weak-lift buzzer:** a soft, continuous tone between your buzzer setting (−0.3 to 0.0 m/s) and the climb threshold. It starts quiet and low, then gets louder and higher until the beeps take over.
- **Auto take-off and landing:** the vario stays silent on the ground and turns on when you take off. After landing it asks "LANDED?".
- **Settings saved** in EEPROM. Leaving the menu asks YES/NO before saving.
- **4 main screens:**
  - vario and altitude;
  - max altitudes;
  - max/min vario;
  - max/min average.

## Hardware

| Part | Connection |
|---|---|
| Arduino Nano (ATmega328P) | |
| BMP180 barometer | I2C (A4 SDA, A5 SCL) |
| MPU6050 accelerometer/gyro | I2C (A4 SDA, A5 SCL) |
| Waveshare LCD1602 I2C display | I2C (A4 SDA, A5 SCL) |
| Passive buzzer | D3 |
| Buttons 1 / 2 / 3 | D4 / D5 / D6, with 10K pull-down resistors (pressed = HIGH) |
| Green / red LED | D7 / D8 |

## Libraries

- `BMP180` (iarduino)
- `Waveshare_LCD1602`
- `Bounce2`
- `SBB_CLICK`
- `Wire`, `EEPROM` (built in)

## Menu pages

1. Sea pressure / altitude
2. Climb threshold
3. Sink threshold
4. LEDs
5. Button beep
6. Volume, base frequency (500–1200 Hz) and buzzer
7. Response speed
8. Average time (5–30 s or OFF)
9. Auto start
10. Factory reset
11. Accelerometer calibration
12. Info (version, temperature, loop time, free RAM, accelerometer and barometer status)

## Calibrating the accelerometer (do this once)

1. Open menu page 11.
2. Lay the vario still on a table on each of its 6 sides (like a dice): screen up, screen down, left, right, top end, bottom end.
3. On each side, press button 3 and don't touch it for about 2 seconds.
4. After all 6 sides the screen shows **"Cal saved"**. The calibration is stored separately, so a factory reset doesn't erase it.

## Files

- `nx_vario/nx_vario.ino` — the current firmware. Open this folder in the Arduino IDE.
- `tools/mpu_test_v1/` — test sketch for the MPU6050.
- `docs/wiring.png` — wiring diagram.

The commit history keeps every version, from the original sketch up to the current one.

## Credits

The original NX-VARIO design, firmware and 3D-printable case are by **XONIX2004**:
- Profile: https://www.thingiverse.com/XONIX2004

This project started from his sketch. The Russian comments were translated to English, and a lot was added since: the accelerometer fusion, buzzer, menus, saved settings and auto take-off.
