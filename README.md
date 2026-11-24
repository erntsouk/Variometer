# NX-VARIO

Arduino Nano paragliding vario with baro/IMU fusion. Based on the original NX-VARIO by [XONIX2004](https://www.thingiverse.com/XONIX2004).

![Wiring](docs/wiring.png)

## Features

- **Fusion:** BMP180 + MPU6050, using a third-order complementary filter whose states are altitude, climb rate and accelerometer bias. The gain is selectable in the menu (Response). Response is about 50 ms.
- **Orientation:** the gravity vector is tracked with the gyro, and the accelerometer only corrects it when |a| is within 0.05 g of g0. This prevents false climb in turns.
- **Calibration:** 6-face accelerometer calibration (offset and scale per axis), stored in its own EEPROM block.
- **Audio:** Timer2 fast PWM on D3, with volume set by duty cycle.
  - Climb: pitch and beep rate follow the vario.
  - Sink: a continuous tone that drops with the sink rate.
  - Weak-lift buzzer: a continuous tone from the set value (−0.3 to 0.0 m/s, or OFF) up to the climb threshold. Pitch and volume both ramp up toward the climb tone.
- **Average:** 5–30 s, or OFF (live value).
- **Auto take-off and landing:** take-off at 10 m altitude change, or more than 0.5 m/s for 3 s. Audio is muted on the ground.
- **Settings:** stored in EEPROM, with a YES/NO save prompt when leaving the menu.

## Hardware

| Part | Pins |
|---|---|
| Arduino Nano (ATmega328P) | |
| BMP180, MPU6050, LCD1602 I2C | A4 SDA / A5 SCL |
| Passive buzzer | D3 (OC2B) |
| Buttons 1 / 2 / 3 | D4 / D5 / D6, 10K pull-down (pressed = HIGH) |
| LED green / red | D7 / D8 |

### Display

The original NX-VARIO uses a standard 1602 LCD with a PCF8574 I2C backpack (0x27). This build uses a **Waveshare LCD1602 I2C** module instead (AiP31068L controller, 0x3E, no backlight control). Pick the firmware that matches your display:

- Waveshare LCD1602 I2C → `nx_vario/`
- Standard 1602 + PCF8574 backpack → `nx_vario_lcd_i2c/`

The case in `case/` is built around the original display.

## Libraries

`BMP180` (iarduino), `Bounce2`, `SBB_CLICK`, plus the display library:
- `nx_vario/` — Waveshare LCD1602 I2C (AiP31068L, 0x3E): `Waveshare_LCD1602`
- `nx_vario_lcd_i2c/` — standard 1602 + PCF8574 backpack (original NX-VARIO display, 0x27): `LiquidCrystal_I2C`

## Build notes

- Flash use is about 28.4 KB of 30 KB on the old bootloader, so there isn't much room left.
- Values marked `CAN BE CHANGED!!!` are the tuning parameters.
- Run the calibration (menu 11) once after assembly. Place the vario still on each face and press button 3.

## Files

- `nx_vario/` — firmware, Waveshare display.
- `nx_vario_lcd_i2c/` — same firmware, standard I2C 1602 display.
- `tools/mpu_test_v1/` — MPU6050 test sketch.
- `docs/wiring.png` — wiring diagram.
- `case/` — 3D-printable case (STL).

The commit history contains every version from the original sketch onward.

## Case

`case/` — NX-Vario case v1.2 by XONIX2004 ([thing:3321624](https://www.thingiverse.com/thing:3321624)), CC BY. Parts: top, bottom, battery cap, sensor cap, button caps. Print **without supports**. The battery can be swapped without opening the case.

The case is designed around the **standard 1602 LCD module (80 × 36 mm PCB) with a PCF8574 I2C backpack**, which is the display the original NX-VARIO uses. The window, PCB mounting and internal clearance are made for that module.

The Waveshare LCD1602 I2C used in this build is a different module, with a different PCB size, mounting and connector position. It does not drop into this case as is. Either:
- use the standard 1602 + backpack with `nx_vario_lcd_i2c/` and print the case unchanged, or
- adapt the top part (window and mounts) to the Waveshare module.

## Credits

The original NX-VARIO design, firmware and case are by **XONIX2004**: https://www.thingiverse.com/XONIX2004

Original build write-up: https://www.instructables.com/DIY-Arduino-Variometer-for-Paragliding/
