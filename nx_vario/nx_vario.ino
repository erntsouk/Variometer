// NOTE!!! Lines whose comment says "CAN BE CHANGED!!!" really can be changed. If the logic is unclear, contact the original code author: xonix2004@list.ru

#include <BMP180.h>                 // bmp180 sensor library, description here (http://blog.iarduino.ru/page/novaya-biblioteka-dlya-d..)
#include <Wire.h>                   // I2C library
#include <EEPROM.h>                // memory for storing settings
#include <Waveshare_LCD1602.h>      // driver for the Waveshare I2C display
#include <Bounce2.h>                // library to debounce the buttons
#include <SBB_CLICK.h>              // library for button hold (long press)
#include <math.h>                   // math library (here for the logarithm used to calculate altitude above sea level)
#define MMHG_TO_HPA 1.33322         // the sensor gives pressure in mmHg; convert to hPa
BMP180 sensor;                      // 
class VarioLCD : public Waveshare_LCD1602, public Print {
public:
  VarioLCD(uint8_t cols, uint8_t rows) : Waveshare_LCD1602(cols, rows) {}
  size_t write(uint8_t value) override { write_char(value); return 1; }
  void createChar(uint8_t location, uint8_t charmap[]) { customSymbol(location, charmap); }
};

VarioLCD lcd(16, 2);
#define switchPin1 4                // button 1 pin (D4, as in the wiring diagram)
#define switchPin2 5                // button 2 pin (D5, as in the wiring diagram)
#define switchPin3 6                // button 3 pin
#define p 3                         // buzzer pin
#define gLED 7                      // GREEN LED pin
#define rLED 8                      // RED LED pin

Bounce b1 = Bounce();               // create Bounce object 1 (button debouncing)
Bounce b2 = Bounce();               // create Bounce object 2 (button debouncing)
Bounce b3 = Bounce();               // create Bounce object 3 (button debouncing)

SBB_CLICK Buttom1(switchPin1,1);    // create the object with (button 1 pin, seconds to hold) - for long press detection
SBB_CLICK Buttom2(switchPin2,1);    // create the object with (button 2 pin, seconds to hold) - for long press detection
SBB_CLICK Buttom3(switchPin3,1);    // create the object with (button 3 pin, seconds to hold) - for long press detection

float vario_down = -1.1;            // SINK threshold for the buzzer         - CAN BE CHANGED only together with TrDNcount, take values from the treshDN array!!!
float vario_up = 0.1;               // CLIMB threshold for the buzzer          - CAN BE CHANGED only together with TrUPcount, take values from the treshUP array!!!
float beep;                         // variable for vario sound output
float Beep_period;                  // variable for vario sound output
float vario = 0;                    // vario value
float varioAvg = 0;                 // averaged vario value for the screen
float avgAlt[31];                   // altitude snapshots once per second (enough for 30 seconds)
byte avgIdx = 0;                    // position in the ring buffer
byte avgFill = 0;                   // how many snapshots have been collected
unsigned long avgTime = 0;          // time of the last snapshot
byte AvgCount = 1;                  // selected averaging time (index into avgSec)
const byte avgSec[6] = {5, 10, 15, 20, 30, 0};   // CAN BE CHANGED!!! averaging time options, seconds (0 = OFF, live value)
float maxVario = 0;                 // maximum vario value
float minVario = 0;                 // minimum vario value
float maxVarioAvg = 0;              // maximum averaged vario value
float minVarioAvg = 0;              // minimum averaged vario value
float presSea = 1013.25;            // sea level pressure in hPa (DEFAULT=1013.25)   - CAN BE CHANGED!!!
float presTMP = 0;                  // variable for manual pressure entry
float tempo = millis();             // tempo (float) = millis() - counts time since start in milliseconds

///// SENSOR FUSION: MPU6050 ACCELEROMETER + BMP180 BAROMETER /////
const byte MPU = 0x68;              // MPU6050 I2C address
boolean mpuOK = false;              // is the MPU6050 answering (without it the vario runs on the barometer only)
float gbx = 0, gby = 0, gbz = 0;    // gyro bias in deg/s, measured at start
float g0 = 1.0;                     // gravity as this accelerometer reads it, g, measured at start
float ux = 0, uy = 0, uz = 1;       // "up" direction in the sensor's own axes (unit vector)
float hEst = 0;                     // fused altitude, m
float vEst = 0;                     // fused climb rate, m/s
float bEst = 0;                     // accelerometer bias estimate, m/s2
float fOmega = 0.7;                 // filter speed, 1/s (set by the Response page)
unsigned long fusTime = 0;          // time of the last filter update, us
boolean fusInit = false;            // has the filter started
#define TILT_K 0.02                 // CAN BE CHANGED!!! how fast "up" follows the accelerometer (0.01 - 0.05)
#define TILT_GATE 0.05              // CAN BE CHANGED!!! only trust the accelerometer for "up" when it reads within this of 1 g (stops turns fooling it)
float calOff[3] = {0, 0, 0};        // accelerometer offsets, g (from the 6-side calibration)
float calSc[3] = {1, 1, 1};         // accelerometer scale (from the 6-side calibration)
boolean calDone = false;            // is a calibration loaded
float calP[6][3];                   // averaged readings of the 6 sides while calibrating
byte calMask = 0;                   // which sides are measured (bit per side)
byte calMsg = 0;                    // 0 - nothing, 1 - saved, 2 - failed, 3 - side already done
#define CAL_ADDR 64                 // EEPROM address of the calibration (separate from the settings)
#define CAL_MAGIC 0xCA11            // marks a valid calibration
struct AccCal {uint16_t magic; float off[3]; float sc[3];};

float treshUP[5] = {0.1, 0.4, 0.5, 0.6, 0.7};                           // climb thresholds                       - CAN BE CHANGED!!! related variables: vario_up and TrUPcount
float treshDN[5] = {-0.5, -0.7, -0.9, -1.1, -1.3};                      // sink thresholds                        - CAN BE CHANGED!!! related variables: vario_down and TrDNcount


int altTmp = 0;                     // altitude when zeroed with BUTTON 1 (altitude = sensor altitude - altitude saved at that point (altTmp))
int altTmpAdd = 0;                  // extra altitude when zeroed with BUTTON 1 (altitude = sensor altitude - altitude saved at that point (altTmpAdd))
int altSEATmp = 0;                  // altitude needed when resetting the maximum on MAIN SCREEN 2
int altSEATmpZero = 0;              // variable needed for altitude above sea level
int altitude = 0;                   // altitude
int altitudeAdd = 0;                // extra altitude
int altitudeSEA = 0;                // altitude above sea level
int altdif = 0;                     // difference between altitude above take-off and pressure altitude
int maxAltSea = 0;                  // maximum altitude above sea level
int maxAltStt = 0;                  // maximum altitude above take-off
int maxAltAdd = 0;                  // maximum extra altitude

char s0 = '1';                      // digit of the sea level pressure (used while entering the value)
char s1 = '0';                      // digit of the sea level pressure (used while entering the value)
char s2 = '1';                      // digit of the sea level pressure (used while entering the value)
char s3 = '3';                      // digit of the sea level pressure (used while entering the value)
char s4 = '2';                      // digit of the sea level pressure (used while entering the value)

char presSTR[7] = "1013.2";         // string used for manual pressure entry
char t0 = '0';                      // altitude digits (for the altitude setting)
char t1 = '0';                      //
char t2 = '0';                      //
char t3 = '0';                      //
char altSTR[5] = "0000";            // altitude string being edited
int altSet = 0;                     // entered altitude in metres
boolean altChange = false;          // altitude entry mode on/off
#define FREQ_MIN 500                // CAN BE CHANGED!!! lowest base frequency, Hz
#define FREQ_MAX 1200               // CAN BE CHANGED!!! highest base frequency, Hz
unsigned int baseFreq = 600;        // vario base sound frequency, Hz
byte volView = 0;                   // volume page view: 0 - volume, 1 - base frequency, 2 - buzzer
boolean buzEdit = false;            // buzzer is being set
byte BuzCount = 1;                  // selected buzzer value (index into buzVals)
const float buzVals[5] = {-0.3, -0.2, -0.1, 0.0, 99};   // CAN BE CHANGED!!! buzzer start, m/s (99 = OFF)
boolean volEdit = false;            // volume is being set
boolean freqEdit = false;           // base frequency is being entered
boolean B3Freq = false;             // button 3 pressed while entering frequency
char f0 = '0';                      // base frequency digits
char f1 = '6';                      //
char f2 = '0';                      //
char f3 = '0';                      //
char freqSTR[5] = "0600";           // base frequency string
boolean altMode = false;            // false - set pressure in hPa, true - set altitude in metres
boolean B3Alt = false;              // button 3 pressed in altitude entry mode

boolean longpress = false;          // long press check
boolean beep_ON = true;             // vario sound on/off
boolean soundManual = false;        // sound switched manually - automatic control leaves it alone
boolean button_beep_ON = true;      // button sound on/off                                   - CAN BE CHANGED!!!
boolean LED_ON = false;             // LED indication on/off                                 - CAN BE CHANGED!!!
boolean gLEDtmp = false;            // green LED on/off
boolean rLEDtmp = false;            // red LED on/off
boolean MainMenu = false;           // are we in the menu
boolean askSave = false;            // is the "save settings?" screen shown
boolean saveChoice = true;          // selected answer to that question (YES/NO)
boolean askReset = false;           // is the "Reset all?" question shown
boolean resetChoice = false;        // selected answer (default NO)
byte infoCount = 0;                 // which item the info page shows
#define FW_VERSION "v37"            // program version shown on the info page
boolean presChange = false;         // pressure entry mode on/off
boolean B3Pres = false;             // raise the pressure digit by one when the button is pressed
boolean TresUPchange = false;       // flag to change the climb threshold with button 3
boolean TresDNchange = false;       // flag to change the sink threshold with button 3
boolean AlarmFlag = false;          // fall alarm flag
boolean Alarm = false;              // fall alarm ON/OFF flag
boolean AfterAlarmFlag = false;     // flag to clear characters left after the alarm
boolean AltDifTmpFlag = false;      // flag: was the altitude reset or not (one for all altitudes, the code picks it by cnt)
byte RespCount = 3;                 // selected vario response speed (index into respOmega)
const float respOmega[5] = {2.0, 1.4, 1.0, 0.7, 0.5};   // CAN BE CHANGED!!! filter speed for each Response level (higher = trusts the barometer more)
float loopMs = 35;                  // average loop time in ms, measured outside the menu
unsigned long lastLoop = 0;         // time of the previous loop

static char outtemp[8];             // char array for temperature rounded to tenths
static char outvario[8];            // char array for vario value rounded to tenths
static char outalt[8];              // char array for altitude
static char outTrUP[8];             // char array for the climb threshold being edited (in MENU)
static char outTrDN[8];             // char array for the sink threshold being edited (in MENU)
static char outtime[8];             // char array for time

unsigned long timeZero = 0;         // variable to reset the timer (MUST be unsigned long!!!)
boolean AutoStart = true;           // automatic take-off and landing detection
boolean inFlight = false;           // are we flying
boolean askLanded = false;          // is the "LANDED?" question shown
boolean landChoice = true;          // selected answer to that question
boolean landDismiss = false;        // after answering NO, do not ask again until there is movement
boolean groundInit = false;         // has the first ground altitude been read
float groundAlt = 0;                // ground reference altitude
float landAlt = 0;                  // altitude when the quiet period started
unsigned long varioTime = 0;        // when steady climb/sink started
unsigned long quietTime = 0;        // when the quiet period started
unsigned long flightMs = 0;         // flight duration, ms

byte TrUPcount = 0;                 // counter to choose the climb threshold from the array    - CAN BE CHANGED only together with vario_up, take the index from treshUP!!!
byte TrDNcount = 3;                 // counter to choose the sink threshold from the array      - CAN BE CHANGED only together with vario_down, take the index from treshDN!!!
byte cnt = 4;                       // button 1 press counter
byte volCount = 3;                  // buzzer volume: 0-OFF, 1-LOW, 2-MED, 3-MAX
const byte volDuty[4] = {0, 2, 6, 15};  // CAN BE CHANGED!!! PWM duty cycle for each volume, %
unsigned long toneOff = 0;          // time when the sound should stop
byte menuCount = 0;                 // menu page counter
byte windowCount = 0;               // main screen counter
byte symCount = 0;                  // number of the digit being entered (pressure entry)
byte symbolINT = 0;                 // integer digit for pressure entry

byte degree[8]                      // degree symbol
{ 
  B11100, 
  B10100, 
  B11100, 
  B00000, 
  B00011, 
  B00100, 
  B00100, 
  B00011, 
}; 
byte arrow_up[8]                    // up arrow symbol
{ 
  B00100, 
  B00100, 
  B01110, 
  B01110, 
  B01110, 
  B11111, 
  B11111, 
  B11111, 
}; 
byte arrow_down[8]                  // down arrow symbol
{ 
  B11111, 
  B11111, 
  B11111, 
  B01110, 
  B01110, 
  B01110, 
  B00100, 
  B00100, 
}; 
byte A_1[8]                         // ALTITUDE ABOVE SEA LEVEL symbol
{ 
  B01110, 
  B10001, 
  B11111, 
  B10001, 
  B00000, 
  B00000, 
  B01010, 
  B10101, 
}; 
byte A_2[8]                         // ALTITUDE ABOVE TAKE-OFF symbol
{ 
  B01110, 
  B10001, 
  B11111, 
  B10001, 
  B00000, 
  B00000, 
  B11111, 
  B11111, 
}; 
byte A_3[8]                         // ALTITUDE 3 symbol
{ 
  B01110, 
  B10001, 
  B11111, 
  B10001, 
  B00000, 
  B00100, 
  B01110, 
  B11111, 
}; 
byte arrow_e_up[8]                  // empty up arrow symbol
{ 
  B00100, 
  B00100, 
  B01010, 
  B01010, 
  B01010, 
  B10001, 
  B10001, 
  B11111,
}; 
byte arrow_e_down[8]                // empty down arrow symbol
{ 
  B11111, 
  B10001, 
  B10001, 
  B01010, 
  B01010, 
  B01010, 
  B00100, 
  B00100,
}; 

///// INITIAL SETUP /////
///// SAVED SETTINGS (EEPROM) /////
struct SavedSettings {
  uint16_t magic;
  byte TrUP, TrDN, vol, avg, resp;
  uint16_t base;
  byte buz;
  boolean autoStart;
  byte led, btnBeep, thermal;
  float presSea;
};
#define SETTINGS_MAGIC 0xA560       // change this number to reset the saved settings

void applySettings()
{
  vario_up = treshUP[TrUPcount];
  vario_down = treshDN[TrDNcount];
  applyResponse();                  // and the response speed
}

void loadSettings()
{
  SavedSettings s;
  EEPROM.get(0, s);
  if (s.magic == SETTINGS_MAGIC && s.TrUP < 5 && s.TrDN < 5 && s.vol < 4 && s.avg < 5 && s.resp < 5 && s.buz < 6 && s.base >= FREQ_MIN && s.base <= FREQ_MAX && s.presSea > 300.0 && s.presSea < 1200.0)
    {
      TrUPcount = s.TrUP; TrDNcount = s.TrDN; volCount = s.vol; AvgCount = (s.avg < 6) ? s.avg : 1;
      LED_ON = s.led; button_beep_ON = s.btnBeep; RespCount = s.resp; AutoStart = s.autoStart; baseFreq = s.base; BuzCount = (s.buz < 5) ? s.buz : 4;
      presSea = s.presSea;
    }
}

void saveSettings()
{
  SavedSettings s;
  s.magic = SETTINGS_MAGIC;
  s.TrUP = TrUPcount; s.TrDN = TrDNcount; s.vol = volCount; s.avg = AvgCount;
  s.led = LED_ON; s.btnBeep = button_beep_ON; s.resp = RespCount; s.autoStart = AutoStart; s.base = baseFreq; s.buz = BuzCount;
  s.presSea = presSea;
  EEPROM.put(0, s);                 // only changed bytes are written
}

///// SETTINGS BACKUP ON MENU ENTRY /////
byte bkTrUP, bkTrDN, bkVol, bkAvg;  // copies of the settings when the menu was opened
boolean bkLED, bkBtn, bkAuto;       //
byte bkResp;                        //
unsigned int bkBase;                //
byte bkBuz;                         //
float bkPres;                       //

void backupSettings()               // remember the settings when the menu opens
{
  bkTrUP = TrUPcount; bkTrDN = TrDNcount; bkVol = volCount; bkAvg = AvgCount;
  bkLED = LED_ON; bkBtn = button_beep_ON; bkResp = RespCount; bkAuto = AutoStart; bkBase = baseFreq; bkBuz = BuzCount;
  bkPres = presSea;
}

void restoreSettings()              // put the settings back as they were when the menu opened
{
  TrUPcount = bkTrUP; TrDNcount = bkTrDN; volCount = bkVol; AvgCount = bkAvg;
  LED_ON = bkLED; button_beep_ON = bkBtn; RespCount = bkResp; AutoStart = bkAuto; baseFreq = bkBase; BuzCount = bkBuz;
  presSea = bkPres;
  applySettings();
  avgFill = 0;                      // start averaging again
}

///// VARIO RESPONSE SPEED /////
void applyResponse()                // apply the selected response speed
{
  fOmega = respOmega[RespCount];    // the filter picks it up on its next update
}

///// MPU6050 ACCESS /////
void mpuWrite(byte reg, byte val)   // write one register of the MPU6050
{
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

boolean mpuRead(int16_t *v)         // read ax, ay, az, temp, gx, gy, gz in one go
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {return false;}
  if (Wire.requestFrom(MPU, (byte)14) != 14) {return false;}
  for (byte i = 0; i < 7; i++)
    {
      byte hi = Wire.read();
      byte lo = Wire.read();
      v[i] = (int16_t)((hi << 8) | lo);
    }
  return true;
}

void mpuInit()                      // wake up and calibrate - the vario must be still
{
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() != 0) {mpuOK = false; return;}   // not connected
  mpuWrite(0x6B, 0x00);             // wake up
  mpuWrite(0x1C, 0x08);             // accelerometer range +-4 g
  mpuWrite(0x1B, 0x00);             // gyro range +-250 deg/s
  mpuWrite(0x1A, 0x03);             // built-in low-pass filter ~44 Hz
  delay(100);                       //
  int16_t v[7];
  float sx = 0, sy = 0, sz = 0, sgx = 0, sgy = 0, sgz = 0;
  byte n = 0;
  for (byte i = 0; i < 100; i++)    // average 100 readings (~0.6 s)
    {
      if (mpuRead(v)) {sx += v[0]; sy += v[1]; sz += v[2]; sgx += v[4]; sgy += v[5]; sgz += v[6]; n++;}
      delay(5);
    }
  if (n < 50) {mpuOK = false; return;}
  sx = (sx / (n * 8192.0) - calOff[0]) / calSc[0];          // g  (+-4 g range = 8192 per g), calibration applied
  sy = (sy / (n * 8192.0) - calOff[1]) / calSc[1];
  sz = (sz / (n * 8192.0) - calOff[2]) / calSc[2];
  gbx = sgx / (n * 131.0); gby = sgy / (n * 131.0); gbz = sgz / (n * 131.0);   // deg/s
  g0 = sqrt(sx*sx + sy*sy + sz*sz); // gravity as this sensor reads it
  if ((g0 < 0.5) || (g0 > 1.5)) {mpuOK = false; return;}    // nonsense - do not use it
  ux = sx / g0; uy = sy / g0; uz = sz / g0;                  // "up" at start
  mpuOK = true;
}

///// SENSOR FUSION /////
// Accelerometer = fast but drifts, barometer = slow but honest.
// A 3rd-order complementary filter (fixed-gain Kalman) uses the accelerometer for quick changes,
// lets the barometer correct the altitude, and learns the accelerometer's bias on the way.
void fusionUpdate(float hBaro)      // called every time the barometer has a new altitude
{
  unsigned long now = micros();
  if (fusInit == false) {hEst = hBaro; vEst = 0; bEst = 0; fusTime = now; fusInit = true; return;}
  float dt = (now - fusTime) / 1000000.0;
  fusTime = now;
  if ((dt <= 0) || (dt > 0.5)) {hEst = hBaro; return;}       // after a long pause just resync
  float az = 0;                     // vertical acceleration, m/s2 (0 if there is no MPU6050)
  int16_t v[7];
  if (mpuOK && mpuRead(v))
    {
      float ax = (v[0] / 8192.0 - calOff[0]) / calSc[0];     // g, calibration applied
      float ay = (v[1] / 8192.0 - calOff[1]) / calSc[1];
      float a_z = (v[2] / 8192.0 - calOff[2]) / calSc[2];
      float wx = (v[4] / 131.0 - gbx) * 0.0174533;           // rad/s, bias removed
      float wy = (v[5] / 131.0 - gby) * 0.0174533;
      float wz = (v[6] / 131.0 - gbz) * 0.0174533;
      float nx = ux - (wy*uz - wz*uy) * dt;                  // turn "up" with the gyro
      float ny = uy - (wz*ux - wx*uz) * dt;
      float nz = uz - (wx*uy - wy*ux) * dt;
      float am = sqrt(ax*ax + ay*ay + a_z*a_z);
      if (fabs(am - g0) < TILT_GATE)                         // only when it is not being shaken or turned: nudge toward measured gravity
        {nx += (ax/am - nx) * TILT_K; ny += (ay/am - ny) * TILT_K; nz += (a_z/am - nz) * TILT_K;}
      float nm = sqrt(nx*nx + ny*ny + nz*nz);
      ux = nx / nm; uy = ny / nm; uz = nz / nm;
      az = ((ax*ux + ay*uy + a_z*uz) - g0) * 9.81;           // acceleration along "up", gravity removed
    }
  float e = hBaro - hEst;           // how far the barometer disagrees
  float w = fOmega;
  hEst += (vEst + 3*w*e) * dt;
  vEst += (az - bEst + 3*w*w*e) * dt;
  bEst -= w*w*w*e * dt;
  vario = vEst;
}

///// ACCELEROMETER 6-SIDE CALIBRATION /////
// Lay the vario still on each of its 6 sides and press button 3 on each.
// Opposite sides give each axis's offset and scale (works even if the sensor is mounted a bit crooked).
void calLoad()                      // read a saved calibration at start
{
  AccCal c;
  EEPROM.get(CAL_ADDR, c);
  if (c.magic != CAL_MAGIC) {return;}
  for (byte k = 0; k < 3; k++)
    {if ((fabs(c.off[k]) > 0.25) || (c.sc[k] < 0.9) || (c.sc[k] > 1.1)) {return;}}
  for (byte k = 0; k < 3; k++) {calOff[k] = c.off[k]; calSc[k] = c.sc[k];}
  calDone = true;
}

void calSide()                      // measure the side the vario is lying on now
{
  if (mpuOK == false) {calMsg = 2; return;}
  int16_t v[7];
  float a[3] = {0, 0, 0};
  byte n = 0;
  for (byte i = 0; i < 100; i++)
    {
      if (mpuRead(v)) {a[0] += v[0]; a[1] += v[1]; a[2] += v[2]; n++;}
      delay(5);
    }
  if (n < 50) {calMsg = 2; return;}
  for (byte k = 0; k < 3; k++) {a[k] /= n * 8192.0;}         // raw g, no correction
  byte ax = 0;                                               // which axis points up or down
  for (byte k = 1; k < 3; k++) {if (fabs(a[k]) > fabs(a[ax])) {ax = k;}}
  byte side = ax * 2 + (a[ax] < 0 ? 1 : 0);                  // X+ X- Y+ Y- Z+ Z-
  if (calMask & (1 << side)) {calMsg = 3; return;}           // this side is already done
  for (byte k = 0; k < 3; k++) {calP[side][k] = a[k];}
  calMask |= (1 << side);
  calMsg = 0;
  if (calMask == 0x3F) {calSolve(); calMask = 0;}            // all 6 done
}

void calSolve()                     // offset = middle of the two opposite sides, scale = length of the gravity vector
{
  AccCal c;
  c.magic = CAL_MAGIC;
  for (byte k = 0; k < 3; k++)      // offsets first: X from X+/X-, Y from Y+/Y-, Z from Z+/Z-
    {c.off[k] = (calP[k*2][k] + calP[k*2+1][k]) / 2;}
  for (byte k = 0; k < 3; k++)      // then scales: on both sides of an axis the corrected vector should be 1 g long
    {
      float len = 0;
      for (byte sd = k*2; sd < k*2+2; sd++)
        {
          float sq = 0;
          for (byte j = 0; j < 3; j++) {float d = calP[sd][j] - c.off[j]; sq += d*d;}
          len += sqrt(sq) / 2;
        }
      c.sc[k] = len;
    }
  for (byte k = 0; k < 3; k++)
    {if ((fabs(c.off[k]) > 0.25) || (c.sc[k] < 0.9) || (c.sc[k] > 1.1)) {calMsg = 2; return;}}   // nonsense - keep the old one
  for (byte k = 0; k < 3; k++) {calOff[k] = c.off[k]; calSc[k] = c.sc[k];}
  EEPROM.put(CAL_ADDR, c);          // saved straight away - it belongs to the sensor, not to the settings
  calDone = true;
  calMsg = 1;
  mpuInit();                        // re-measure "up" and the gyro with the new calibration
  fusInit = false;                  // restart the filter
}

///// AUTO TAKE-OFF AND LANDING /////
#define TO_ALT 10.0                 // CAN BE CHANGED!!! altitude change to detect take-off, m
#define TO_VARIO 0.5                // CAN BE CHANGED!!! vario threshold to detect take-off, m/s
#define TO_SEC 3                    // CAN BE CHANGED!!! how many seconds this threshold must hold
#define LAND_VARIO 0.2              // CAN BE CHANGED!!! vario threshold for being still, m/s
#define LAND_ALT 3.0                // CAN BE CHANGED!!! allowed altitude change while still, m
#define LAND_SEC 60                 // CAN BE CHANGED!!! seconds of stillness before asking about landing

void takeOff()                      // take-off detected
{
  inFlight = true;                  //
  altTmp = int(groundAlt);          // altitude above take-off is measured from the launch point
  timeZero = millis();              // start the flight timer
  flightMs = 0;                     //
  maxVario = 0; minVario = 0;       // reset vario max/min
  maxVarioAvg = 0; minVarioAvg = 0; //
  maxAltSea = 0; maxAltStt = 0; maxAltAdd = 0;   // and the maximum altitudes
  quietTime = 0;                    //
  landAlt = sensor.alti;            //
  beep_ON = true;                  // turn on vario sound
  soundManual = false;             //
  triple_beep();                    // take-off signal
}

void landed()                       // landing confirmed
{
  inFlight = false;                 // flight timer stops at its value
  askLanded = false;                //
  groundAlt = sensor.alti;          // track the ground altitude again
  varioTime = 0;                    //
  quietTime = 0;                    //
  landDismiss = false;              //
  beep_ON = false;                 // on the ground the vario sound is off
  soundManual = false;             //
  SSL_beep();                       // landing signal
  lcd.clear();                      //
}

void flightDetect()                 // called once per second
{
  if (groundInit == false) {groundAlt = sensor.alti; landAlt = sensor.alti; groundInit = true;}
  if ((MainMenu == false) && (soundManual == false))                                 // silent on the ground, sound in flight
    {beep_ON = ((AutoStart == false) || (inFlight == true));}                        //
  if (AutoStart == false) {return;} //
  unsigned long now = millis();     //
  if (inFlight == false)            // ===== ON THE GROUND =====
    {
      groundAlt += (sensor.alti - groundAlt) * 0.05;    // slowly follow the ground altitude (time constant ~20 s)
      if (fabs(vario) > TO_VARIO)   // steady climb or sink...
        {if (varioTime == 0) {varioTime = now;}}        // ... start timing
      else {varioTime = 0;}         //
      if ((fabs(sensor.alti - groundAlt) > TO_ALT) ||                                  // altitude changed...
          ((varioTime != 0) && (now - varioTime >= (unsigned long)TO_SEC*1000)))       // ... or the vario stayed above threshold
        {takeOff();}                //
    }
  else                              // ===== IN FLIGHT =====
    {
      if (fabs(vario) > TO_VARIO) {landDismiss = false;}   // movement again - can ask again
      if ((fabs(vario) < LAND_VARIO) && (fabs(sensor.alti - landAlt) < LAND_ALT))
        {
          if (quietTime == 0) {quietTime = now;}           // start timing the quiet period
          if ((now - quietTime >= (unsigned long)LAND_SEC*1000) && (askLanded == false)
              && (landDismiss == false) && (MainMenu == false))
            {askLanded = true; landChoice = true; triple_beep(); lcd.clear();}          // ask "LANDED?"
        }
      else {quietTime = 0; landAlt = sensor.alti;}         // quiet period broken
    }
}

///// LEAVE MENU /////
void exitMenu()
{
  askSave = false;                    // remove the question screen
  askReset = false;                   // and the reset question
  altChange = false;                  // turn off altitude entry
  volEdit = false;                    // turn off volume setting
  buzEdit = false;                    // and buzzer setting
  calMask = 0;                        // a half-done calibration is dropped
  calMsg = 0;                         //
  freqEdit = false;                   // and frequency entry
  MainMenu = false;                   // close the menu
  menuCount = 0;                      // reset the menu page counter
  symCount = 0;                       // reset the digit counter
  if (presChange == true)             // if leaving while editing pressure...
    {
      presChange = false;             // ... turn off the mode
      presTMP = presSea;              // ... restore the previous value
      presToStr(presTMP);   //
    }
  if (saveChoice == true) {saveSettings();}   // YES - save settings to memory
  else                                        // NO - put the settings back as they were when the menu opened
    {
      restoreSettings();              //
    }
  presTMP = presSea;                  //
  presToStr(presTMP);       //
  beep_ON = ((AutoStart == false) || (inFlight == true));   // vario sound turns on only in flight
  soundManual = false;                //
  lcd.clear();                        // clear the screen
}

void fmtNum(char *buf, float v, byte width, byte dec)   // number as right-aligned text, 0 or 1 decimal (replaces dtostrf)
{
  char tmp[10];
  byte n = 0;
  boolean neg = (v < 0);
  if (neg) {v = -v;}
  long x = (long)(v * (dec ? 10 : 1) + 0.5);
  if (x == 0) {neg = false;}                                 // no "-0.0"
  if (dec) {tmp[n++] = '0' + x % 10; x /= 10; tmp[n++] = '.';}
  do {tmp[n++] = '0' + x % 10; x /= 10;} while ((x > 0) && (n < 8));
  if (neg) {tmp[n++] = '-';}
  byte i = 0;
  while (i + n < width) {buf[i++] = ' ';}
  while (n > 0) {buf[i++] = tmp[--n];}
  buf[i] = 0;
}

void presToStr(float v)             // pressure as text "1013.2" (4 digits + 1 after the point), written into presSTR
{
  char buf[10];
  fmtNum(buf, v, 6, 1);
  for (byte i = 0; buf[i] == ' '; i++) {buf[i] = '0';}
  for (byte i = 0; i < 6; i++) {presSTR[i] = buf[i];}
  presSTR[6] = 0;
}

void put4(char *dst, char a, char b, char c, char d)   // four digit characters into a text
{
  dst[0] = a; dst[1] = b; dst[2] = c; dst[3] = d; dst[4] = 0;
}

void setup() 
{ 
  // Serial.begin(9600);               // set the serial port to 9600 baud

  loadSettings();                   // read the saved settings from EEPROM
  presTMP = presSea;                // temporary pressure (for calculations) = default pressure (presSea is used in the altitude formula)
  presToStr(presTMP);        // convert pressure from float to String
  applySettings();                  // apply thresholds from the saved settings (vario_up, vario_down)
  backupSettings();                 // first settings snapshot (in case NO is answered before the menu is ever opened)
  
  pinMode(p, OUTPUT);               // pin 3 as output (buzzer)
  pinMode(gLED, OUTPUT);            // pin 7 as output (green LED)
  pinMode(rLED, OUTPUT);            // pin 8 as output (red LED)
  
  pinMode(switchPin1, INPUT);       // pin 4 as input (button)
  digitalWrite(switchPin1, LOW);    // disable the internal pull-up; an external 10k resistor is used
  b1.attach(switchPin1);            // Bounce object listens to the button on switchPin1
  b1.interval(5);                   // debounce time 5 ms

  pinMode(switchPin2, INPUT);       // pin 5 as input (button)
  digitalWrite(switchPin2, LOW);    // disable the internal pull-up; an external 10k resistor is used
  b2.attach(switchPin2);            // Bounce object listens to the button on switchPin2
  b2.interval(5);                   // debounce time 5 ms

  pinMode(switchPin3, INPUT);       // pin 6 as input (button)
  digitalWrite(switchPin3, LOW);    // disable the internal pull-up; an external 10k resistor is used
  b3.attach(switchPin3);            // Bounce object listens to the button on switchPin3
  b3.interval(5);                   // debounce time 5 ms
  
  lcd.init();                       // LCD init
  lcd.setCursor(0,0);               //
  lcd.print(F(" HELLO ERNESTO "));    // first line of the start screen
  lcd.setCursor(0,1);               //
  lcd.print(F("BARO SENSOR TEST"));    // second line of the start screen
  delay(1000);                      // 1 second delay
    for (int i = 0; i<2; i++)             // blink the LEDs at start. Supposedly for a higher purpose, really it just looks cool! =)
      {                                   //
        digitalWrite(gLED, HIGH);         // green LED on at start
        digitalWrite(rLED, HIGH);         // red LED on at start
        delay(50);                        //
        digitalWrite(gLED, LOW);          // green LED off at start
        digitalWrite(rLED, LOW);          // red LED off at start
        delay(100);                       //
      }                                   //
  lcd.clear();                      // clear screen
  delay(5);                         // give the display time to finish clearing
  lcd.setCursor(3,0);               //
  lcd.print(F("def. pres."));          // label "default pressure"
  lcd.setCursor(3,1);               //
  lcd.print(presSea,1);             // show the start pressure in hPa
  lcd.print(F(" hPa"));                //
  calLoad();                        // accelerometer calibration from memory (if there is one)
  mpuInit();                        // start and calibrate the MPU6050 (~0.7 s, keep the vario still)
  delay(1300);                      // rest of the 2 second pause
  playTone((2500),70);                // double beep at start
  beepDelay(150);                       //
  playTone((2500),70);                //
  beepDelay(70);                   //
  lcd.clear();                      // clear the screen before the main window
  
  sensor.begin();                   // bmp180 init. Start altitude in metres can be given in the brackets
  
  lcd.createChar(1, degree);        // create degree symbol
  lcd.createChar(2, arrow_up);      // create up arrow symbol
  lcd.createChar(3, arrow_down);    // create down arrow symbol
  lcd.createChar(4, A_1);           // create A_1 symbol
  lcd.createChar(5, A_2);           // create A_2 symbol
  lcd.createChar(6, A_3);           // create A_3 symbol
  lcd.createChar(7, arrow_e_up);    // create empty up arrow symbol
  lcd.createChar(0, arrow_e_down);  // create empty up arrow symbol
} 

///// LCD OUTPUT /////
void lcd_out()                                                          // screen output function, code by nuclear_XONIX (Iliushenko Andrei)
{                                                                       // 
if (askLanded == true)                                                  // landing question screen
  {                                                                     //
    lcd.setCursor(0,0);                                                 //
    lcd.print(F("LANDED?         "));                                      //
    lcd.setCursor(0,1);                                                 //
    if (landChoice == true) {lcd.print(F("  > YES     no  "));}            // YES selected
    else                    {lcd.print(F("    yes   > NO  "));}            // NO selected
    return;                                                             //
  }                                                                     //

if (Alarm == false)                                                     // if the alarm is off
{                                                                       //
if (AfterAlarmFlag == true)                                             // if the after-alarm clear flag is set
  {lcd.clear(); AfterAlarmFlag = false;}                                // clear the screen and reset the flag

///// MAIN WINDOW 1 /////
if (windowCount == 0)                                                   // if the first main screen is selected
{                                                                       //
  
  lcd.setCursor(5,0);                                                   // keep the space after the vario value empty
  lcd.print(' ');                                                       //
      
///// SYMBOL "S" OUTPUT (for Speaker) /////
  lcd.setCursor(9,0);                                                   //
  if (beep_ON == true) {lcd.print(F("S"));}                                // if sound is on, show SOUND symbol ("S")
  else {lcd.print(F(" "));}                                                // otherwise show a blank (" ")

///// VARIO INDICATION OUTPUT (Arrows) /////
    if (vario>0.1)                                                      // if vario is positive (>0.1 to allow for sensor error), fill the up arrow
    {                                                                   //
      lcd.setCursor(0,0);                                               //
      lcd.write(byte(2));                                               //
    }                                                                   // 
    if (vario<-0.1)                                                     // if vario is negative (<-0.1 to allow for sensor error), fill the down arrow
    {                                                                   //
      lcd.setCursor(0,1);                                               //
      lcd.write(byte(3));                                               //
    }                                                                   // 
    if ((vario>=-0.1) && (vario<=0.1))                                  // if vario is zero (-0.02 to 0, to allow for sensor error), show both empty arrows
    {                                                                   //
      lcd.setCursor(0,0);                                               //
      lcd.write(byte(7));                                               //
      lcd.setCursor(0,1);                                               //
      lcd.write(byte(0));                                               //
    }                                                                   // 

///// VARIO OUTPUT /////
  fmtNum(outvario, varioAvg, 4, 1);                                        // convert vario (float) to a char array rounded to tenths
  lcd.setCursor(1,0);                                                   //
  lcd.print(outvario);                                                  // show vario rounded to tenths

///// TEMPERATURE OUTPUT (C) /////
  fmtNum(outtemp, sensor.temp, 5, 1);                                   // convert sensor.temp to a char array rounded to tenths
  lcd.setCursor(10,1);                                                  //
  lcd.print(outtemp);                                                   // show temperature rounded to tenths
  lcd.write(byte(1));                                                   // degree symbol after the temperature

///// TIME OUTPUT (Min) /////
  lcd.setCursor(11,0);                                                  //
  lcd.print(F("Tm"));                                                      // print Tm (time, minutes)
  fmtNum(outtime, ((flightMs/1000)/60), 3, 0);               // convert elapsed time to a char array rounded to whole numbers
  lcd.print(outtime);                                                   // show time rounded to whole numbers

///// ALT ABOVE SEA OUTPUT (m) /////
if (cnt==4)                                                             // ALTITUDE ABOVE SEA LEVEL
  {                                                                     //
  lcd.setCursor(2,1);                                                   //
  lcd.write(byte(cnt));                                                 // altitude symbol (cnt - counter changed by the button)
  fmtNum(outalt, float(altitudeSEA), 4, 0);                             // convert sensor.temp to a char array rounded to tenths
  lcd.print(outalt);                                                    // show temperature rounded to tenths
  lcd.print(F("m"));                                                       //
  }                                                                     //

///// ALT ABOVE TAKE OFF POINT OUTPUT (m) /////
if (cnt==5)                                                             // ALTITUDE ABOVE TAKE-OFF
  {                                                                     // 
  lcd.setCursor(2,1);                                                   //
  lcd.write(byte(cnt));                                                 // altitude symbol (cnt - counter changed by the button)
  fmtNum(outalt, float(altitude), 4, 0);                                // convert sensor.temp to a char array rounded to tenths
  lcd.print(outalt);                                                    // show temperature rounded to tenths
  lcd.print(F("m"));                                                       //
  }                                                                     //

///// ADDITIONAL ALT OUTPUT (m) /////
if (cnt==6)                                                             // EXTRA ALTITUDE
  {                                                                     //
  lcd.setCursor(2,1);                                                   //
  lcd.write(byte(cnt));                                                 // altitude symbol (cnt - counter changed by the button)
  fmtNum(outalt, float(altitudeAdd), 4, 0);                             // convert sensor.temp to a char array rounded to tenths
  lcd.print(outalt);                                                    // show temperature rounded to tenths
  lcd.print(F("m"));                                                       //
  }                                                                     //
}                                                                       //

///// MAIN WINDOW 2 (Max altitudes) /////
if (windowCount == 1)                                                                   // if the second main screen is selected
  {                                                                                     //
    lcd.setCursor(0,0);                                                                 //
    lcd.print(F("Max Alt: ")); lcd.write(byte(4)); lcd.print(' '); lcd.print(maxAltSea); lcd.print('m');   // title and maximum altitude above sea level
    lcd.setCursor(0,1);                                                                 //
    lcd.write(byte(5)); lcd.print(' '); lcd.print(maxAltStt); lcd.print('m');                           // maximum altitude above take-off
    lcd.setCursor(9,1);                                                                 //
    lcd.write(byte(6)); lcd.print(' '); lcd.print(maxAltAdd); lcd.print('m');                           // maximum extra altitude
  }                                                                                     //

///// Min vario) /////
if (windowCount == 2)                                                   // if the third main screen is selected
  {                                                                     //
    lcd.setCursor(0,0);                                                 //
    lcd.print(F("Max Var:   ")); lcd.print(maxVario);                      // print maximum vario
    lcd.setCursor(0,1);                                                 //
    lcd.print(F("Min Var:   ")); lcd.print(minVario);                      // print minimum vario
  }                                                                     //

///// Min averaged vario) /////
if (windowCount == 3)                                                   // if the fourth screen is selected...
  {                                                                     //
    lcd.setCursor(0,0);                                                 //
    lcd.print(F("Max Avg:   ")); lcd.print(maxVarioAvg);                   // show maximum averaged value
    lcd.setCursor(0,1);                                                 //
    lcd.print(F("Min Avg:   ")); lcd.print(minVarioAvg);                   // show minimum averaged value
  }                                                                     //
}                                                                       //
}                                                                       // end of screen output

///// TONE WITH VOLUME CONTROL /////
// /// Buzzer on pin 3 (OC2B). Timer 2 sets the frequency, PWM duty cycle sets the volume.
void stopTone()                                                         // stop the sound
{
  TCCR2A = 0;                                                           //
  TCCR2B = 0;                                                           //
  digitalWrite(p, LOW);                                                 //
  toneOff = 0;                                                          //
}

void playTone(unsigned int freq, unsigned int dur)                      // sound with given frequency and duration
{
  if (volCount == 0) {stopTone(); return;}                              // volume OFF - stay silent
  if (freq < 70) {freq = 70;}                                           // timer 2 cannot go lower
  if (freq > 5000) {freq = 5000;}                                       //
  byte top;                                                             //
  byte cs;                                                              //
  if (freq < 250) {top = (15625UL / freq) - 1; cs = _BV(CS22) | _BV(CS21) | _BV(CS20);}   // low frequencies: prescaler 1024
  else            {top = (62500UL / freq) - 1; cs = _BV(CS22) | _BV(CS21);}               // the rest: prescaler 256
  OCR2A = top;                                                          // frequency
  OCR2B = (unsigned int)top * volDuty[volCount] / 100;                  // volume
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);                       // fast PWM on OC2B
  TCCR2B = _BV(WGM22) | cs;                                             // TOP = OCR2A, selected prescaler
  toneOff = millis() + dur;                                             // when to stop
}

void toneService()                                                      // called every loop - stops the sound on time
{
  if ((toneOff != 0) && (millis() >= toneOff)) {stopTone();}            //
}

void beepDelay(unsigned int ms)                                         // pause during which the sound still stops on time
{
  unsigned long t = millis() + ms;                                      //
  while (millis() < t) {toneService();}                                 //
}

///// BUTTON BEEP /////
void button_beep()                                                      // BUTTON BEEP
{                                                                       //
  if (button_beep_ON == true) {playTone(1800,30); beepDelay(30);}                        // beep only if button sound is on
}                                                                       //

///// LONG BEEP /////
void long_beep()                                                        // LONG BEEP
{                                                                       //
  if (button_beep_ON == true) {playTone(1800,200); beepDelay(200);}                       // beep only if button sound is on
}                                                                       //

///// TRIPLE BEEP /////
void triple_beep()                                                      // TRIPLE BEEP
{                                                                       //
  if (button_beep_ON == true)                                           // beep only if button sound is on
  {                                                                     //
    playTone(1800,40);                                                    //
    beepDelay(80);                                                          //
    playTone(1800,40);                                                    //
    beepDelay(80);                                                          //
    playTone(1800,40);                                                    // 
  }                                                                     //
}                                                                       //

///// SHORT-SHORT-LONG BEEP /////
void SSL_beep()                                                         // SHORT-SHORT-LONG BEEP
{                                                                       //
  if (button_beep_ON == true)                                           // beep only if button sound is on
  {                                                                     //
    playTone(1800,40);                                                    //
    beepDelay(80);                                                          //
    playTone(1800,40);                                                    //
    beepDelay(40);                                                       //
    beepDelay(80);                                                          //
    playTone(1800,150);                                                   // 
    beepDelay(150);                                                      //
  }
}

///// MAIN MENU /////
void Menu()                                                                       //
{                                                                                 //
  if (askSave == true)                                                            // save settings question screen
    {
      lcd.setCursor(0,0);                                                           //
      lcd.print(F("Save settings?  "));                                                //
      lcd.setCursor(0,1);                                                           //
      if (saveChoice == true) {lcd.print(F("  > YES     no  "));}                      // YES selected
      else                    {lcd.print(F("    yes   > NO  "));}                      // NO selected
      return;                                                                       //
    }

  if (askReset == true)                                                          // factory reset question screen
    {
      lcd.setCursor(0,0);                                                           //
      lcd.print(F("Reset all?      "));                                                //
      lcd.setCursor(0,1);                                                           //
      if (resetChoice == true) {lcd.print(F("  > YES     no  "));}                     // YES selected
      else                     {lcd.print(F("    yes   > NO  "));}                     // NO selected
      return;                                                                       //
    }

  lcd.setCursor(0,0);                                                             //
  lcd.print(F("MENU ")); lcd.print(menuCount+1); lcd.print(F("   "));                                   // print MENU and the page number on the top line
    if (menuCount == 0) {SeaPresChange();}                                        // menu page 1 - sea pressure / altitude
    else if (menuCount == 1) {TresholdUP();}                                      // menu page 2 - climb threshold
    else if (menuCount == 2) {TresholdDN();}                                      // menu page 3 - sink threshold
    else if (menuCount == 3) {LED_light();}                                       // menu page 4 - LEDs ON/OFF
    else if (menuCount == 4) {ButtonBeep();}                                      // menu page 5 - button sound ON/OFF
    else if (menuCount == 5) {VolumeSet();}                                       // menu page 6 - buzzer volume
    else if (menuCount == 6) {Response();}                                        // menu page 7 - vario response speed
    else if (menuCount == 7) {AvgTime();}                                         // menu page 8 - averaging time
    else if (menuCount == 8) {AutoStartSet();}                                     // menu page 9 - auto take-off detection
    else if (menuCount == 9) {FactoryResetPage();}                                // menu page 10 - factory reset
    else if (menuCount == 10) {CalPage();}                                        // menu page 11 - accelerometer calibration
    else if (menuCount == 11) {InfoPage();}                                       // menu page 12 - info
}                                                                                 //

///// SET ALTITUDE, PRESSURE IS CALCULATED /////
char nextDigit(char c)              // next digit (after 9 back to 0)
{
  byte v = int(c);                  //
  v++;                              //
  if (v == 58) {v = 48;}            //
  return char(v);                   //
}

void altToDigits(int v)             // split altitude into four digits
{
  if (v < 0) {v = 0;}               //
  if (v > 9999) {v = 9999;}         //
  t0 = char(48+(v/1000)%10);        //
  t1 = char(48+(v/100)%10);         //
  t2 = char(48+(v/10)%10);          //
  t3 = char(48+v%10);               //
}

void presToDigits(float v)          // split pressure into five characters
{
  presToStr(v);           //
  s0 = presSTR[0];                  //
  s1 = presSTR[1];                  //
  s2 = presSTR[2];                  //
  s3 = presSTR[3];                  //
  s4 = presSTR[5];                  //
}


///// SEA PRESSURE VALUE CHANGE PROCEDURE /////
void SeaPresChange()                                                              // combined page: sea pressure / altitude
{
  if (altMode == false)                                                           // ===== "PRESSURE" MODE =====
    {
      if (presChange == true)                                                     // if editing...
        {
          if (B3Pres == true)                                                     // ... and button 3 pressed, change the selected digit
            {
              if (symCount == 0) {s0 = nextDigit(s0);}                            //
              if (symCount == 1) {s1 = nextDigit(s1);}                            //
              if (symCount == 2) {s2 = nextDigit(s2);}                            //
              if (symCount == 3) {s3 = nextDigit(s3);}                            //
              if (symCount == 5) {s4 = nextDigit(s4);}                            //
              B3Pres = false;                                                     //
            }
          presSTR[0] = s0; presSTR[1] = s1; presSTR[2] = s2; presSTR[3] = s3; presSTR[4] = '.'; presSTR[5] = s4; presSTR[6] = 0;                                   // build the pressure string
          presTMP = (s0-'0')*1000.0 + (s1-'0')*100 + (s2-'0')*10 + (s3-'0') + (s4-'0')/10.0;                                            //
          presSea = presTMP;                                                      // apply immediately - altitude updates instantly
          lcd.setCursor(10,0);                                                    //
          lcd.print(F("     "));                                                     // erase the old cursor
          lcd.setCursor((10+symCount),0);                                         //
          lcd.print(F("v"));                                                         // draw the cursor over the selected digit
        }
      else                                                                        // if not editing - show current pressure
        {
          presToDigits(presSea);                                                  // refresh digits from presSea (may have changed via altitude)
          presTMP = presSea;                                                      //
          lcd.setCursor(10,0);                                                    //
          lcd.print(F("      "));                                                    // erase the cursor
        }
      lcd.setCursor(0,1);                                                         //
      lcd.print(F("Sea pres. "));                                                    // draw "Sea pres."
      lcd.setCursor(10,1);                                                        //
      lcd.print(presSTR);                                                         // show pressure in hPa
    }
  else                                                                            // ===== "ALTITUDE" MODE =====
    {
      if (altChange == true)                                                      // if editing...
        {
          if (B3Alt == true)                                                      // ... and button 3 pressed, change the selected digit
            {
              if (symCount == 0) {t0 = nextDigit(t0);}                            //
              if (symCount == 1) {t1 = nextDigit(t1);}                            //
              if (symCount == 2) {t2 = nextDigit(t2);}                            //
              if (symCount == 3) {t3 = nextDigit(t3);}                            //
              B3Alt = false;                                                      //
            }
          put4(altSTR, t0, t1, t2, t3);                                           // build the altitude string
          altSet = (t0-'0')*1000 + (t1-'0')*100 + (t2-'0')*10 + (t3-'0');                                                //
          presSea = (sensor.pres*MMHG_TO_HPA)*exp(float(altSet)/(29.21*(sensor.temp+273.15)));   // calculate pressure from altitude immediately
          presToDigits(presSea);                                                  // and refresh the pressure digits
          presTMP = presSea;                                                      //
          lcd.setCursor(11,0);                                                    //
          lcd.print(F("    "));                                                      // erase the old cursor
          lcd.setCursor((11+symCount),0);                                         //
          lcd.print(F("v"));                                                         // draw the cursor over the selected digit
        }
      else                                                                        // if not editing - show current altitude
        {
          altToDigits(altitudeSEA);                                               //
          put4(altSTR, t0, t1, t2, t3);                                           //
          lcd.setCursor(11,0);                                                    //
          lcd.print(F("     "));                                                     // erase the cursor
        }
      lcd.setCursor(0,1);                                                         //
      lcd.print(F("Set alt.   "));                                                   // draw "Set alt."
      lcd.setCursor(11,1);                                                        //
      lcd.print(altSTR); lcd.print('m');                                                      // show altitude in metres
    }
}

///// SETTING TRESHOLD FOR CLIMB /////
void TresholdUP()                                                                 //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print(F("Tr.hold UP "));                                                       // draw "Tr.hold UP "
  lcd.print(F("     "));                                                             // erase old characters
    if (TresUPchange == true)                                                     // if button 3 was pressed...
      {                                                                           //
        vario_up = treshUP[TrUPcount];                                            // set vario_up (used for the climb sound) to the next value from the array
        TresUPchange = false;                                                     // clear the button 3 press
      }                                                                           // 
  lcd.setCursor(12,1);                                                            // 
  fmtNum(outTrUP, vario_up, 4, 1);                                               // convert vario_up from float to a char array for the screen
  lcd.print(outTrUP);                                                     // show the climb threshold
}                                                                                 //

///// SETTING TRESHOLD FOR DESCENT /////
void TresholdDN()                                                                 // 
{                                                                                 // 
  lcd.setCursor(0,1);                                                             // 
  lcd.print(F("Tr.hold DN "));                                                       // draw "Tr.hold DN "
  lcd.print(F("     "));                                                             // erase old characters
    if (TresDNchange == true)                                                     // if button 3 was pressed...
      {                                                                           //
        vario_down = treshDN[TrDNcount];                                          // set vario_down (used for the sink sound) to the next value from the array
        TresDNchange = false;                                                     // clear the button 3 press
      }                                                                           // 
  lcd.setCursor(12,1);                                                            // 
  fmtNum(outTrDN, vario_down, 4, 1);                                             // convert vario_down from float to a char array for the screen
  lcd.print(outTrDN);                                                     // show the sink threshold
}                                                                                 //

///// OFF /////
void LED_light()                                                                  //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print(F("Light      "));                                                       // draw "Light"
  lcd.setCursor(13,1);                                                            //
    if (LED_ON == true) {lcd.print(F("ON "));}                                       // if LEDs are on, show ON
    else {lcd.print(F("OFF"));}                                                      // if off, show OFF
}                                                                                 //

///// VARIO MODE SET /////
///// OFF /////
void AutoStartSet()                                                               //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print(F("Auto start "));                                                       // draw "Auto start"
  lcd.setCursor(12,1);                                                            //
  if (AutoStart == true) {lcd.print(F("  ON"));}                                     //
  else {lcd.print(F(" OFF"));}                                                       //
}                                                                                 //

///// FACTORY RESET PAGE /////
void FactoryResetPage()                                                           //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print(F("Factory reset   "));                                                  // button 3 asks "Reset all?"
}                                                                                 //

void factoryReset()                 // all settings back to default, saved straight away
{
  TrUPcount = 0; TrDNcount = 3;     // thresholds
  volCount = 3; baseFreq = 600; BuzCount = 1;   // sound (buzzer -0.2)
  AvgCount = 1; RespCount = 3;      // averaging and response
  LED_ON = false; button_beep_ON = true; AutoStart = true;
  presSea = 1013.25;                // standard pressure
  presTMP = presSea;                //
  presToStr(presSea);     //
  applySettings();                  //
  avgFill = 0;                      //
  saveSettings();                   // write to memory
  backupSettings();                 // so the exit question cannot undo it
  SSL_beep();                       // confirmation beep
}

///// ACCELEROMETER CALIBRATION PAGE /////
void CalPage()                                                                    //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  if (calMsg == 1) {lcd.print(F("Cal saved       ")); return;}                    //
  if (calMsg == 2) {lcd.print(F("Cal failed      ")); return;}                    //
  if (calMsg == 3) {lcd.print(F("Side done, next ")); return;}                    //
  lcd.print(F("Cal "));                                                           // sides measured so far, ".." = still to do
  const char sides[] = "XXYYZZ";                                                  //
  for (byte i = 0; i < 6; i++)                                                    //
    {                                                                             //
      if (calMask & (1 << i)) {lcd.print(sides[i]); lcd.print((i & 1) ? '-' : '+');}
      else {lcd.print(F(".."));}                                                  //
    }                                                                             //
}                                                                                 //

///// INFO PAGE /////
extern int __heap_start, *__brkval; // used to measure free RAM
int freeRam()                       // free RAM in bytes
{
  int v;                            //
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void InfoPage()                                                                   // button 3 shows the next item
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  if (infoCount == 0) {lcd.print(F("Version   ")); lcd.print(FW_VERSION); lcd.print(F("   "));}
  else if (infoCount == 1) {lcd.print(F("Temp     ")); lcd.print(sensor.temp,1); lcd.print(F("C  "));}
  else if (infoCount == 2) {lcd.print(F("Loop      ")); lcd.print((int)loopMs); lcd.print(F("ms  "));}
  else if (infoCount == 3) {lcd.print(F("Free RAM  ")); lcd.print(freeRam()); lcd.print(F("B  "));}
  else if (infoCount == 4)                                                        // accelerometer status
    {
      lcd.print(F("Accel "));
      if (mpuOK == false) {lcd.print(F("none      "));}
      else if (calDone) {lcd.print(F("OK cal    "));}
      else {lcd.print(F("OK no cal "));}
    }
  else                                                                            // barometer status
    {
      lcd.print(F("Baro  "));
      Wire.beginTransmission(0x77);                                               // is the BMP180 answering?
      if (Wire.endTransmission() == 0) {lcd.print(F("OK        "));} else {lcd.print(F("none      "));}
    }
}                                                                                 //

///// AVERAGING TIME /////
void AvgTime()                                                                    //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print(F("Average    "));                                                       // draw "Average"
  lcd.setCursor(12,1);                                                            //
  if (avgSec[AvgCount] == 0) {lcd.print(F(" OFF")); return;}                       // OFF - no averaging
  if (avgSec[AvgCount] < 10) {lcd.print(F("  "));}                                   //
  else {lcd.print(F(" "));}                                                          //
  lcd.print(avgSec[AvgCount]);                                                    // show the averaging time
  lcd.print(F("s"));                                                                 //
}                                                                                 //

///// BUZZER VOLUME /////
void VolumeSet()                                                                  // volume / base frequency page
{                                                                                 //
  if (volView == 2)                                                               // ===== "BUZZER" VIEW =====
    {
      lcd.setCursor(0,1);                                                         //
      lcd.print(F("Buzzer     "));                                                //
      lcd.setCursor(11,1);                                                        //
      if (buzVals[BuzCount] > 50) {lcd.print(F("  OFF"));}                        //
      else                                                                        //
        {
          if (buzVals[BuzCount] >= 0) {lcd.print(F(" "));}                        //
          lcd.print(buzVals[BuzCount],1);                                         // e.g. -0.2
          lcd.print(F(" "));                                                      //
        }
      lcd.setCursor(10,0);                                                        //
      if (buzEdit == true) {lcd.print(F("   v  "));}                              // cursor over the value
      else {lcd.print(F("      "));}                                              //
    }
  else if (volView == 0)                                                          // ===== "VOLUME" VIEW =====
    {
      lcd.setCursor(0,1);                                                         //
      lcd.print(F("Volume      "));                                                  //
      lcd.setCursor(12,1);                                                        //
      if (volCount == 0) {lcd.print(F(" OFF"));}                                     //
      else if (volCount == 1) {lcd.print(F(" LOW"));}                                //
      else if (volCount == 2) {lcd.print(F(" MED"));}                                //
      else {lcd.print(F(" MAX"));}                                                   //
      lcd.setCursor(10,0);                                                        //
      if (volEdit == true) {lcd.print(F("    v "));}                                 // cursor over the value
      else {lcd.print(F("      "));}                                                 //
    }
  else                                                                            // ===== "BASE FREQUENCY" VIEW =====
    {
      if (freqEdit == true)                                                       // if editing...
        {
          if (B3Freq == true)                                                     // ... and button 3 pressed, change the digit
            {
              if (symCount == 0) {f0 = nextDigit(f0);}                            //
              if (symCount == 1) {f1 = nextDigit(f1);}                            //
              if (symCount == 2) {f2 = nextDigit(f2);}                            //
              if (symCount == 3) {f3 = nextDigit(f3);}                            //
              B3Freq = false;                                                     //
            }
          put4(freqSTR, f0, f1, f2, f3);                                          //
          lcd.setCursor(10,0);                                                    //
          lcd.print(F("      "));                                                    // erase the old cursor
          lcd.setCursor((10+symCount),0);                                         //
          lcd.print(F("v"));                                                         // cursor over the selected digit
        }
      else                                                                        //
        {
          freqToDigits(baseFreq);                                                 //
          lcd.setCursor(10,0);                                                    //
          lcd.print(F("      "));                                                    //
        }
      lcd.setCursor(0,1);                                                         //
      lcd.print(F("Base freq "));                                                    //
      lcd.setCursor(10,1);                                                        //
      lcd.print(freqSTR); lcd.print(F("Hz"));                                                    // show the frequency
    }
}                                                                                 //

void freqToDigits(unsigned int v)   // split the frequency into four digits
{
  f0 = char(48+(v/1000)%10);        //
  f1 = char(48+(v/100)%10);         //
  f2 = char(48+(v/10)%10);          //
  f3 = char(48+v%10);               //
  put4(freqSTR, f0, f1, f2, f3);    //
}

///// VARIO MODE SET /////
///// RESPONSE SPEED SET /////
void Response()                                                                   //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print(F("Resp "));                                                             // draw "Resp"
  lcd.print(RespCount+1);                                                         // show level 1...5
  lcd.setCursor(10,1);                                                            //
  float win = 1.0 / fOmega;                                          // averaging window = points x loop time, seconds
  if (win < 10) {lcd.print(F(" "));}                                                 //
  lcd.print(win,1);                                                               // show the window in seconds
  lcd.print(F("s "));                                                                //
}                                                                                 //

///// OFF /////
void ButtonBeep()                                                                 //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print(F("But. sound "));                                                       // draw "Button sound"
  lcd.setCursor(13,1);                                                            //
  if (button_beep_ON == true) {lcd.print(F("ON "));}                                 // if sound is on, show ON
  else {lcd.print(F("OFF"));}                                                        // if off, show OFF
}                                                                                 //

///// MAIN CYCLE /////
void loop()                                                                               //
{                                                                                         //
toneService();                                                          // stop the sound when it is time
{                                                                       // measure loop time (only outside the menu, so it matches flying)
  unsigned long nowL = millis();                                        //
  if ((MainMenu == false) && (lastLoop != 0))                           //
    {loopMs += ((float)(nowL - lastLoop) - loopMs) * 0.02;}             // slow smoothing - steady value
  lastLoop = nowL;                                                      //
}
if (sensor.read(3))                                                                       // check for data from the sensor
{                                                                                         // read the bmp180. Accuracy 0 to 3 in the brackets, 3 is maximum
if (vario>-0.3) {Alarm = false;}                                                          // check fall alarm ON/OFF

///// BUTTON 1 SHORT PRESS /////
if ((b1.update() == 1) && (b1.read() == LOW) && (b1.fell() == 1) && (longpress == false))   // if there was an event, BUTTON 1 is pressed and no long press registered (a short click)
  {                                                                                       //
    button_beep();                                                                        // button press beep
    if (askLanded == true)                                                                // if the landing question is shown...
      {
        if (landChoice == true) {landChoice = false;}                                     // ... move the YES/NO choice
        else {landChoice = true;}                                                         //
      }
    else if (MainMenu == false)                                                           // ... and if we are not in the MENU ...
      {                                                                                   //
        MainMenu = true;                                                                  // menu mode turns on
        backupSettings();                                                         // remember settings so that NO can restore them
          s0 = presSTR[0];                                                                // s0 = first pressure character
          s1 = presSTR[1];                                                                // s1 = second pressure character
          s2 = presSTR[2];                                                                // s2 = third pressure character
          s3 = presSTR[3];                                                                // s3 = fourth pressure character
          s4 = presSTR[5];                                                                // s4 = fifth pressure character
        beep_ON = false;                                                                  // turn off beep_ON (vario sound does not work in the MENU)
        lcd.clear();                                                                      // clear screen on button press
      }                                                                                   //
    else                                                                                  // ... if we are already in the menu
      {
        if (askReset == true)                                                             // if the reset question is shown...
          {
            if (resetChoice == true) {resetChoice = false;}                               // toggle YES/NO
            else {resetChoice = true;}                                                    //
          }
        else
        if (askSave == true)                                                              // if the save question is shown...
          {
            if (saveChoice == true) {saveChoice = false;}                                 // ... toggle YES/NO
            else {saveChoice = true;}                                                     //
          }
        else if ((presChange == false) && (altChange == false) && (volEdit == false) && (freqEdit == false) && (buzEdit == false))                           // otherwise go BACK one menu page
          {
            if (menuCount == 0) {menuCount = 12;}                                          // from the first page go to the last
            menuCount -= 1;                                                               //
            lcd.clear();                                                                  //
          }
      }
  }                                                                                       //

///// BUTTON 1 LONG PRESS /////
if (Buttom1.click())                                                                      // if button 1 is held for more than 1 second... (long press)
  {                                                                                       //
    long_beep();                                                                          // long beep
    if (askLanded == true) {}                                                             // long presses do nothing during the landing question
    else if (MainMenu == false)                                                           // ... and if we are not in the MENU ...
      {                                                                                   //
        lcd.setCursor(13,0);                                                              // RESET TIME with a long press of BUTTON 1
        lcd.print(F("   "));                                                                 // erase the old time value
        timeZero = millis();                                                              // set the variable used to reset the time
        flightMs = 0;                                                                     // reset the flight timer display
      }                                                                                   // 
    else                                                                                  // ... if we are in the menu
      {
        if ((askSave == false) && (askReset == false))                                                             // ask whether to save settings
          {
            askSave = true;                                                               // show the question screen
            saveChoice = true;                                                            // default - YES
            lcd.clear();                                                                  //
          }
      }
    longpress = true;                                                                     // set to true, i.e. a long press happened
  }                                                                                       //
if (b1.fell() == 1) {longpress = false;}                                                  // clear the long press flag when the button is released
Buttom1.release();                                                                        // must be written after click(), otherwise it runs only once

///// BUTTON 2 SHORT PRESS /////
if ((b2.update() == 1) && (b2.read() == LOW) && (b2.fell() == 1) && (longpress == false))   // if there was an event, BUTTON 2 is pressed and no long press registered (short click)
  {                                                                                       //
    button_beep();                                                                        // button press beep
    lcd.clear();                                                                          // clear screen on button press
    if (askLanded == true)                                                                // if the landing question is shown...
      {
        if (landChoice == true) {landChoice = false;}                                     // ... move the YES/NO choice
        else {landChoice = true;}                                                         //
      }
    else if (MainMenu == false)                                                           // ... and if we are not in the MENU ...
      {                                                                                   // 
        if (windowCount == 0)                                                             // if we are on the first main screen
          {                                                                               //
            cnt+=1;                                                                       // increase the counter by one
            if (cnt==7)                                                                   // if the counter reached 7...
              {                                                                           // ... set it to 4
                cnt=4;                                                                    // values 4 to 6 are used because the altitude symbols are numbered 4 to 6
              }                                                                           //
          }                                                                               //
      }                                                                                   //
    else if (askReset == true)                                                        // if the reset question is shown...
      {
        if (resetChoice == true) {resetChoice = false;}                               // toggle YES/NO
        else {resetChoice = true;}                                                    //
      }
    else if (askSave == true)                                                         // if the save question is shown...
      {
        if (saveChoice == true) {saveChoice = false;}                                 // toggle YES/NO
        else {saveChoice = true;}                                                     //
      }
    else
      {                                                                                   //
        if (presChange == true)                                                           // if editing pressure...
          {                                                                               //
            symCount += 1;                                                                // ... move the cursor over the pressure digits
            if (symCount == 4) {symCount += 1;}                                           // skip the decimal point position
            if (symCount == 6) {symCount = 0;}                                            // after the last digit - back to the first
          }                                                                               //
        else if (freqEdit == true)                                                        // if editing frequency...
          {                                                                               //
            symCount += 1;                                                                // ... move the cursor over the frequency digits
            if (symCount == 4) {symCount = 0;}                                            //
          }                                                                               //
        else if (volEdit == true) {}                                                      // no paging while setting volume
        else if (buzEdit == true) {}                                                      // or the buzzer
        else if (altChange == true)                                                       // if editing altitude...
          {                                                                               //
            symCount += 1;                                                                // ... move the cursor over the altitude digits
            if (symCount == 4) {symCount = 0;}                                            // after the last digit - back to the first
          }                                                                               //
        else                                                                              // otherwise go to the next menu page
          {                                                                               //
            menuCount += 1;                                                               //
            if (menuCount == 12) {menuCount = 0;}                                         // after the last page - back to the first
          }                                                                               //
      }                                                                                   //
  }                                                                                       // 

///// BUTTON 2 LONG PRESS /////
if (Buttom2.click())                                                                      // if button 2 is held for more than 1 second... (long press)
  {                                                                                       // 
    SSL_beep();                                                                           // call the SHORT-SHORT-LONG BEEP
    if (askLanded == true) {}                                                             // long presses do nothing during the landing question
    else if (MainMenu == false)                                                           // ... and if we are not in the MENU ...
      {                                                                                   // RESET ALTITUDES with a long press of BUTTON 2
        if (cnt == 4)                                                                     // action for SEA LEVEL altitude (zeroing)
          {                                                                               //
            altSEATmp = altitudeSEA;                                                      // set altSEATmp to the calculated altitude
            presSea = sensor.pres*MMHG_TO_HPA;   // pressure mmHg -> hPa
            presToStr(presSea);                                                    // print the pressure into the string shown in the MENU
            saveSettings();                                                               // save pressure to memory
          }                                                                               //
        if (cnt == 5)                                                                     // action for TAKE-OFF altitude (zeroing)
          {                                                                               //
            altTmp = sensor.alti;                                                         // set altTmp to the sensor altitude
          }                                                                               //
        if (cnt == 6)                                                                     // action for EXTRA altitude (zeroing)
          {                                                                               //
            altTmpAdd = sensor.alti;                                                      // set altTmpAdd to the sensor altitude
          }                                                                               //
        AltDifTmpFlag = true;                                                             // set the flag: altitude was reset
      }                                                                                   //
    else if ((askSave == false) && (askReset == false))                                                                                  // ... if we are in the MENU after all ...
      {                                                                                   //
        if (menuCount == 5)                                                               // on the volume / base frequency page
          {                                                                               //
            if (volView == 2)                                                             // --- "buzzer" view ---
              {                                                                           //
                if (buzEdit == false) {buzEdit = true;}                                   // start setting
                else {buzEdit = false;}                                                   // finish
                lcd.clear();                                                              //
              }                                                                           //
            else if (volView == 0)                                                        // --- "volume" view ---
              {                                                                           //
                if (volEdit == false) {volEdit = true;}                                   // start setting
                else {volEdit = false; playTone(baseFreq,300); beepDelay(300);}           // finish, test beep
                lcd.clear();                                                              //
              }                                                                           //
            else                                                                          // --- "base frequency" view ---
              {                                                                           //
                if (freqEdit == false)                                                    //
                  {                                                                       //
                    freqEdit = true;                                                      // start entering frequency
                    freqToDigits(baseFreq);                                               //
                    symCount = 0;                                                         //
                  }                                                                       //
                else                                                                      //
                  {                                                                       //
                    freqEdit = false;                                                     // finish entering
                    int v = (f0-'0')*1000 + (f1-'0')*100 + (f2-'0')*10 + (f3-'0');                                              //
                    if (v < FREQ_MIN) {v = FREQ_MIN;}                                     // limit the range
                    if (v > FREQ_MAX) {v = FREQ_MAX;}                                     //
                    baseFreq = v;                                                         //
                    freqToDigits(baseFreq);                                               //
                    symCount = 0;                                                         //
                    playTone(baseFreq,300);                                               // test beep at the new frequency
                    beepDelay(300);                                                       //
                  }                                                                       //
                lcd.clear();                                                              //
              }                                                                           //
          }                                                                               //
        if (menuCount == 0)                                                               // on the first page - enter pressure or altitude
          {                                                                               //
            if (altMode == false)                                                         // --- "pressure" mode ---
              {                                                                           //
                if (presChange == false)                                                  //
                  {                                                                       //
                    presChange = true;                                                    // start pressure entry
                    presToDigits(presSea);                                                //
                    presTMP = presSea;                                                    //
                    symCount = 0;                                                         //
                  }                                                                       //
                else                                                                      //
                  {                                                                       //
                    presChange = false;                                                   // finish entering
                    presSea = presTMP;                                                    //
                    symCount = 0;                                                         //
                    altSEATmp = 0;                                                        //
                  }                                                                       //
              }                                                                           //
            else                                                                          // --- "altitude" mode ---
              {                                                                           //
                if (altChange == false)                                                   //
                  {                                                                       //
                    altChange = true;                                                     // start altitude entry
                    altToDigits(altitudeSEA);                                             // start from the current altitude above sea level
                    symCount = 0;                                                         //
                  }                                                                       //
                else                                                                      //
                  {                                                                       //
                    altChange = false;                                                    // finish entering
                    symCount = 0;                                                         //
                    altSEATmp = 0;                                                        //
                  }                                                                       //
              }                                                                           //
          }                                                                               //
      }                                                                                   //
    longpress = true;                                                                     // set to true, i.e. a long press happened
    lcd.clear();                                                                          // clear screen
  }                                                                                       // 
if (b2.fell() == 1) {longpress = false;}                                                  // clear the long press flag when the button is released
Buttom2.release();                                                                        // must be written after click(), otherwise it runs only once

///// BUTTON 3 SHORT PRESS /////
if ((b3.update() == 1) && (b3.read() == LOW) && (b3.fell() == 1) && (longpress == false))   // if there was an event, BUTTON 3 is pressed and no long press registered (short click)
  {                                                                                       //
    button_beep();                                                                        // button press beep
    if (askLanded == true)                                                                // if the landing question is shown...
      {
        if (landChoice == true) {landed();}                                               // YES - end the flight
        else                                                                              // NO - keep flying
          {
            askLanded = false;                                                            //
            landDismiss = true;                                                           // do not ask again until there is movement
            quietTime = 0;                                                                //
            lcd.clear();                                                                  //
          }
      }
    else if (MainMenu == false)                                                           // ... if we are not in the MENU...
      {                                                                                   // 
        windowCount++;                                                                    // ... switch main screens (increase the screen counter)
        if (windowCount == 4)                                                             // if the counter reached the limit...
          {windowCount = 0;}                                                              // ... reset it
        lcd.clear();                                                                      // clear the screen on every switch
      }                                                                                   // 
    else if (askReset == true)                                                        // if the reset question is shown...
      {
        if (resetChoice == true) {factoryReset();}                                    // YES - back to factory settings
        askReset = false;                                                             // NO - just close the question
        lcd.clear();                                                                  //
      }
    else if (askSave == true)                                                         // if the save question is shown...
      {
        exitMenu();                                                               // ... apply the answer and leave the menu
      }
    else
      {                                                                                   // 
        if (presChange == true)                                                           // ... if pressure editing is also on
          {                                                                               //
            B3Pres = true;                                                                // register the BUTTON 3 press (used in SeaPresChange() to increase the selected digit)
          }                                                                               //
        if (altChange == true)                                                             // ... if altitude entry is on
          {                                                                               //
            B3Alt = true;                                                                 // register the button 3 press
          }                                                                               //
        if ((menuCount == 0) && (presChange == false) && (altChange == false))             // on the first page, not editing...
          {                                                                               //
            if (altMode == true) {altMode = false;}                                       // ... button 3 switches: pressure <-> altitude
            else {altMode = true;}                                                        //
            symCount = 0;                                                                 //
            lcd.clear();                                                                  //
          }                                                                               //
        if (menuCount == 1)                                                               // ...if we are on the climb threshold page
          {                                                                               //
            TresUPchange = true;                                                          //
            TrUPcount++;                                                                  //
            if (TrUPcount == 5) {TrUPcount = 0;}                                          //
          }                                                                               //
        if (menuCount == 2)                                                               // ...if we are on the sink threshold page
          {                                                                               //
            TresDNchange = true;                                                          //
            TrDNcount++;                                                                  //
            if (TrDNcount == 5) {TrDNcount = 0;}                                          //
          }                                                                               //
        if (menuCount == 3)                                                               // ...if we are on the LEDs ON/OFF page
          {                                                                               //
            if (LED_ON == true) {LED_ON = false;}                                         //
            else {LED_ON = true;}                                                         //
          }                                                                               //
        if (menuCount == 4)                                                               // ...if we are on the button sound ON/OFF page
          {                                                                               //
            if (button_beep_ON == true) {button_beep_ON = false;}                         //
            else {button_beep_ON = true;}                                                 //
          }                                                                               //
        if (menuCount == 5)                                                               // ...if we are on the volume / base frequency page
          {                                                                               //
            if (volEdit == true)                                                          // setting volume...
              {                                                                           //
                volCount++;                                                               //
                if (volCount == 4) {volCount = 0;}                                        // after MAX back to OFF
                playTone(baseFreq,150);                                                   // test beep at the new volume
                beepDelay(150);                                                           //
              }                                                                           //
            else if (freqEdit == true) {B3Freq = true;}                                   // entering frequency - change the digit
            else if (buzEdit == true)                                                     // setting the buzzer - next value
              {                                                                           //
                BuzCount++;                                                               //
                if (BuzCount == 5) {BuzCount = 0;}                                        // after OFF back to -0.3
              }                                                                           //
            else                                                                          // otherwise switch view: volume -> frequency -> buzzer
              {                                                                           //
                volView++;                                                                //
                if (volView == 3) {volView = 0;}                                          //
                lcd.clear();                                                              //
              }                                                                           //
          }                                                                               //
        if (menuCount == 6)                                                               // ...if we are on the response page
          {                                                                               //
            RespCount++;                                                                  //
            if (RespCount == 5) {RespCount = 0;}                                          // after the last level - back to the first
            applyResponse();                                                              // apply the new response speed
          }                                                                               //
        if (menuCount == 7)                                                               // ...if we are on the averaging time page
          {                                                                               //
            AvgCount++;                                                                   //
            if (AvgCount == 6) {AvgCount = 0;}                                            // after the last value - back to the first
            avgFill = 0;                                                                  // start collecting again
          }                                                                               //
        if (menuCount == 8)                                                               // ...if we are on the auto take-off page
          {                                                                               //
            if (AutoStart == true) {AutoStart = false;}                                   //
            else {AutoStart = true;}                                                      //
            inFlight = false;                                                             // start detection again
            askLanded = false;                                                            //
            varioTime = 0;                                                                //
            quietTime = 0;                                                                //
            groundInit = false;                                                           //
          }                                                                               //
        if (menuCount == 9)                                                               // ...if we are on the factory reset page
          {                                                                               //
            askReset = true;                                                              // ask first
            resetChoice = false;                                                          // default - NO, for safety
            lcd.clear();                                                                  //
          }                                                                               //
        if (menuCount == 10)                                                              // ...if we are on the calibration page
          {                                                                               //
            if (calMsg == 1) {calMsg = 0;}                                                // after "saved" a press starts a new calibration
            else                                                                          //
              {                                                                           //
                calSide();                                                                // measure the side it is lying on
                if (calMsg == 1) {SSL_beep();}                                            // all 6 done and saved
                else if (calMsg >= 2) {triple_beep();}                                    // failed or side already done
              }                                                                           //
            lcd.clear();                                                                  //
          }                                                                               //
        if (menuCount == 11)                                                              // ...if we are on the info page
          {                                                                               //
            infoCount++;                                                                  // next info item
            if (infoCount == 6) {infoCount = 0;}                                          //
            lcd.clear();                                                                  //
          }                                                                               //
      }                                                                                   //
  }                                                                                       //

///// BUTTON 3 LONG PRESS /////
if (Buttom3.click())                                                                      // if button 3 is held for more than 1 second... (long press)
  {                                                                                       //
    triple_beep();                                                                        // call the TRIPLE BEEP
    if (askLanded == true) {}                                                             // long presses do nothing during the landing question
    else if (MainMenu == false)                                                           // ... if we are not in the MENU...
      {
        if (windowCount == 0)                                                             // if we are on the first main screen
          {                                                                               // TURN vario sound OFF/ON
            if (beep_ON == true) {beep_ON = false;}                                       // if sound is on, turn it off
            else {beep_ON = true;}                                                        // otherwise turn it on
            soundManual = true;                                                            // from now on the pilot controls the sound, not the automatic
          }                                                                               //
        if (windowCount == 1)                                                             // if we are on the second main screen
          {                                                                               // RESET MAXIMUM ALTITUDES
            lcd.clear();                                                                  // clear the screen
            maxAltSea = 0;                                                                // reset maximum altitude above sea level
            maxAltStt = 0;                                                                // reset maximum altitude above take-off
            maxAltAdd = 0;                                                                // reset maximum extra altitude
          }                                                                               //
        if (windowCount == 2)                                                             // if we are on the third main screen
          {                                                                               // RESET VARIO MAX/MIN
            lcd.clear();                                                                  // clear the screen
            maxVario = 0;                                                                 // reset maximum vario
            minVario = 0;                                                                 // reset minimum vario
          }                                                                               //
        if (windowCount == 3)                                                             // if we are on the fourth screen...
          {                                                                               // ...reset averaged max/min
            lcd.clear();                                                                  //
            maxVarioAvg = 0;                                                              //
            minVarioAvg = 0;                                                              //
          }                                                                               //
      }                                                                                   //
    else if ((askSave == false) && (askReset == false))                                                                                  // ... if we are in the MENU after all...
      {                                                                                   // long press of BUTTON 3 in the MENU resets parameters to default
        presSea = 1013.25;                                                                 // the altitude formula variable gets its start value
        presTMP = 1013.25;                                                                 // the temporary pressure variable gets its start value
        s0 = '1';                                                                         // pressure digits get their start values
        s1 = '0';                                                                         // pressure digits get their start values
        s2 = '1';                                                                         // pressure digits get their start values
        s3 = '3';                                                                         // pressure digits get their start values
        s4 = '2';                                                                         // pressure digits get their start values
        altSEATmp = 0;                                                                    // reset the corrected altitude (so that after a sea level zeroing it returns to the pressure-based value)
      }                                                                                   //
    longpress = true;                                                                     // set to true, i.e. a long press happened
  }                                                                                       //
if (b3.fell() == 1) {longpress = false;}                                                  // clear the long press flag when the button is released
Buttom3.release();                                                                        // must be written after click(), otherwise it runs only once

///// ALTITUDE CALCULATING /////
altitude = sensor.alti-altTmp;                                                            // TAKE-OFF ALTITUDE - altitude = sensor altitude minus altitude at the zeroing point
altitudeAdd = sensor.alti-altTmpAdd;                                                      // EXTRA ALTITUDE - altitudeAdd = sensor altitude minus altitude at the zeroing point
altitudeSEA = -29.21*float(log((sensor.pres*MMHG_TO_HPA)/presSea))*(sensor.temp+273.15);                // SEA LEVEL ALTITUDE - from the reference pressure, current pressure and temperature

if ((AutoStart == false) || (inFlight == true)) {flightMs = millis() - timeZero;}   // flight timer runs only in flight

///// AVERAGED VARIO FOR THE SCREEN /////
if (millis() - avgTime >= 1000)                                         // store the altitude once per second
  {
    avgTime = millis();                                                 //
    avgIdx++;                                                           //
    if (avgIdx == 31) {avgIdx = 0;}                                     //
    avgAlt[avgIdx] = sensor.alti;                                       //
    if (avgFill < 31) {avgFill++;}                                      //
    flightDetect();                                                     // check take-off / landing
    byte n = avgSec[AvgCount];                                          // selected averaging time
    if ((n > 0) && (avgFill > n))                                       // when averaging is on and enough snapshots are collected...
      {
        byte old = (avgIdx + 31 - n) % 31;                              // snapshot n seconds ago
        varioAvg = (avgAlt[avgIdx] - avgAlt[old]) / float(n);           // average climb rate over that time
      }
  }

if (avgSec[AvgCount] == 0) {varioAvg = vario;}                         // averaging OFF - show the live value

///// MIN VARIO /////
if (maxVario<vario) {maxVario = vario;}                                 // store maximum vario
if (minVario>vario) {minVario = vario;}                                 // store minimum vario
if (maxVarioAvg < varioAvg) {maxVarioAvg = varioAvg;}               // store maximum averaged value
if (minVarioAvg > varioAvg) {minVarioAvg = varioAvg;}               // store minimum averaged value

///// ASSIGNING VALUES FOR MAX ALTITUDES /////
if (AltDifTmpFlag == true)                                              // 
  {                                                                     //
    if (cnt == 4) {maxAltSea = maxAltSea - altSEATmp;}                  //
    if (cnt == 5) {maxAltStt = maxAltStt - altTmp;}                     //
    if (cnt == 6) {maxAltAdd = maxAltAdd - altTmpAdd;}                  //
    AltDifTmpFlag = false;                                              //
  }                                                                     //
else                                                                    //
  {                                                                     //
    if (maxAltSea < altitudeSEA) {maxAltSea = altitudeSEA;}             //
    if (maxAltStt < altitude) {maxAltStt = altitude;}                   //
    if (maxAltAdd < altitudeAdd) {maxAltAdd = altitudeAdd;}             //
  }                                                                     //

///// SENSOR FUSION /////
tempo = millis();                                                       // time base for the beeps
fusionUpdate(sensor.alti);                                              // vario = fused climb rate (accelerometer + barometer)

///// VARIO SOUND OUTPUT /////
if (beep_ON == true)                                                    // if sound mode is ON...
{                                                                       //
if ((tempo-beep)>Beep_period)                                           // 
{                                                                       //
  beep=tempo;                                                           //
  if (vario>vario_up && vario<15)                                       // climb sound
  {                                                                     //
    Alarm = false;                                                      // turn off the alarm
    gLEDtmp = true;                                                     // green LED on climb
    Beep_period = 350-(vario*80);                                       //
    playTone((baseFreq+(250*vario)),300-(vario*80));                          // play the sound (was 1800)
  }                                                                     //
  else if ((buzVals[BuzCount] < 50) && (vario > buzVals[BuzCount]) && (vario <= vario_up))   // buzzer: weak lift, below the climb threshold
  {                                                                     //
    Beep_period = 40;                                                   // refresh every 40 ms - one continuous tone
    float k = (vario - buzVals[BuzCount]) / (vario_up - buzVals[BuzCount]);   // 0 at the buzzer value ... 1 at the climb threshold
    playTone(baseFreq * (0.6 + 0.4 * k), 150);                          // CAN BE CHANGED!!! pitch rises from 60% of base up to base
    byte d = OCR2B * (0.125 + 0.875 * k);                               // CAN BE CHANGED!!! loudness rises from 1/8 of the beep up to the beep
    OCR2B = (d > 0) ? d : 1;                                            //
  }                                                                     //
  else if ((vario<vario_down))                      // sink sound
  {                                                                     //
    Alarm = false;                                                      // turn off the alarm
    rLEDtmp = true;                                                     // red LED on sink
    Beep_period=40;                                                     // update pitch every 40 ms (continuous tone)
    playTone(constrain(int(baseFreq+(baseFreq/5.0)*vario),80,(int)baseFreq),150);   // continuous sink tone: the stronger the sink, the lower the tone
  }                                                                     //
}                                                                       //
}                                                                       //
else                                                                    // ... otherwise just show "B" next to the vario value and light the LEDs
{                                                                       //
if ((tempo-beep)>Beep_period)                                           // 
{                                                                       //
  beep=tempo;                                                           //
  if (vario>vario_up && vario<15 )                                      // 15 picked at random
  {                                                                     //
    gLEDtmp = true;                                                     // green LED on climb
  }                                                                     //
  else if ((vario<vario_down))                      // 
  {                                                                     //
    rLEDtmp = true;                                                     // red LED on sink
  }                                                                     //
}                                                                       //
}                                                                       //

///// OFF FOR VARIO /////
if ((LED_ON == true)&&(gLEDtmp == true))                                // if (LEDs are on) and (green LED allowed)
  {digitalWrite(gLED, HIGH); delay(10); digitalWrite(gLED, LOW);}       // green LED on, wait, LED off (off quickly to save battery)
else if ((LED_ON == false)||(gLEDtmp == false))                         // otherwise, if (LEDs are off) OR (green LED not allowed)
  {digitalWrite(gLED, LOW);}                                            // green LED off
if ((LED_ON == true)&&(rLEDtmp == true))                                // if (LEDs are on) and (red LED allowed)
  {digitalWrite(rLED, HIGH); delay(10); digitalWrite(gLED, LOW);}       // red LED on, wait, LED off (off quickly to save battery)
else if ((LED_ON == true)||(rLEDtmp == false))                          // otherwise, if (LEDs are off) OR (red LED not allowed)
  {digitalWrite(rLED, LOW);}                                            // red LED off


///// START MAIN WINDOW OR MENU /////
if (MainMenu == false)                                                  // If the MENU is OFF...
  {lcd_out();}                                                          // main screen output (NOT MENU)
else                                                                    // ... otherwise ...
  {Menu();}                                                             // clear the screen and run the MENU

///// VALUES TO ZERO IN THE END OF THE CYCLE /////
gLEDtmp = false;                                                        // turn off green LED
rLEDtmp = false;                                                        // turn off red LED
}

///// ERROR MESSAGE IF SOMETHING WRONG WITH BMP180 SENSOR /////
else                                                                    //
  {                                                                     //
    lcd.setCursor(0,0);                                                 //
    lcd.print(F("Sensor error"));                                          // error message if there is no data from the sensor
  }                                                                     //

//Serial.println((String)" samples="+samples+" maxsamples="+maxsamples);
}

