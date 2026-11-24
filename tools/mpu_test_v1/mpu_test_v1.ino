// MPU6050 test - prints accelerometer (g) and gyro (deg/s) to the Serial Monitor (9600 baud)
// Wiring (GY-521 board): VCC -> 5V, GND -> GND, SDA -> A4, SCL -> A5, AD0 and INT not connected

#include <Wire.h>

const byte MPU = 0x68;              // MPU6050 I2C address (AD0 not connected)

void writeReg(byte reg, byte val)   // write one register of the MPU6050
{
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  Wire.beginTransmission(MPU);      // is anything answering at 0x68?
  if (Wire.endTransmission() != 0)
    {
      Serial.println(F("MPU6050 NOT FOUND at 0x68 - check wiring"));
      while (1) {}
    }

  writeReg(0x6B, 0x00);             // wake up (it starts asleep)
  writeReg(0x1C, 0x00);             // accelerometer range +-2 g
  writeReg(0x1B, 0x00);             // gyro range +-250 deg/s
  writeReg(0x1A, 0x03);             // built-in low-pass filter ~44 Hz

  Wire.beginTransmission(MPU);      // read the chip ID
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (byte)1);
  Serial.print(F("Chip ID (WHO_AM_I) = 0x"));
  Serial.println(Wire.read(), HEX);
  Serial.println(F("ax\tay\taz (g)\t\tgx\tgy\tgz (deg/s)"));
}

void loop()
{
  Wire.beginTransmission(MPU);      // read accel, temp and gyro in one go
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (byte)14);
  int16_t v[7];
  for (byte i = 0; i < 7; i++)
    {
      byte hi = Wire.read();
      byte lo = Wire.read();
      v[i] = (int16_t)((hi << 8) | lo);
    }

  Serial.print(v[0] / 16384.0, 2); Serial.print('\t');   // accel X in g
  Serial.print(v[1] / 16384.0, 2); Serial.print('\t');   // accel Y in g
  Serial.print(v[2] / 16384.0, 2); Serial.print(F("\t\t"));   // accel Z in g
  Serial.print(v[4] / 131.0, 1); Serial.print('\t');     // gyro X in deg/s
  Serial.print(v[5] / 131.0, 1); Serial.print('\t');     // gyro Y in deg/s
  Serial.println(v[6] / 131.0, 1);                       // gyro Z in deg/s
  delay(200);
}
