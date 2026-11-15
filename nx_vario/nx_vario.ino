// ВНИМАНИЕ!!! Значения строк, в комментариях к которым написано "МОЖНО МЕНЯТЬ!!!" действительно можно менять. В случае неясности в логике работы, обращаться к автору кода: xonix2004@list.ru

#include <BMP180.h>                 // подключение библиотеки датчика bmp180, описание здесь (http://blog.iarduino.ru/page/novaya-biblioteka-dlya-d..) 
#include <Wire.h>                   // подключение библиотеки для работы с I2C 
#include <EEPROM.h>                // память для хранения настроек
#include <Waveshare_LCD1602.h>      // драйвер встроенного I2C дисплея Waveshare
#include <Bounce2.h>                // подключение библиотеки для избежания дребезга контактов клавиш 
#include <SBB_CLICK.h>              // подключение библиотеки удержания кнопки
#include <math.h>                   // подключение библиотеки для работы с математическими функциями (в данном случае с логарифмом для вычисления высоты на уровне моря)
#define MMHG_TO_HPA 1.33322         // датчик отдаёт давление в мм рт. ст.; переводим в hPa
BMP180 sensor;                      // 
class VarioLCD : public Waveshare_LCD1602, public Print {
public:
  VarioLCD(uint8_t cols, uint8_t rows) : Waveshare_LCD1602(cols, rows) {}
  size_t write(uint8_t value) override { write_char(value); return 1; }
  void createChar(uint8_t location, uint8_t charmap[]) { customSymbol(location, charmap); }
};

VarioLCD lcd(16, 2);
#define switchPin1 5                // пин кнопки 1
#define switchPin2 4                // пин кнопки 2
#define switchPin3 6                // пин кнопки 3
#define p 3                         // пин пищалки
#define gLED 7                      // пин ЗЕЛЕНОГО диода
#define rLED 8                      // пин КРАСНОГО диода

Bounce b1 = Bounce();               // создаём объект Bounce 1 (для удаления дребезга кнопок)
Bounce b2 = Bounce();               // создаём объект Bounce 2 (для удаления дребезга кнопок)
Bounce b3 = Bounce();               // создаём объект Bounce 3 (для удаления дребезга кнопок)

SBB_CLICK Buttom1(5,1);             // создаём экземпляр класса и в параметрах указываем (номер пина кнопки 1, количество секунд нажатия кнопки) - для обнаружения длительного нажатия
SBB_CLICK Buttom2(4,1);             // создаём экземпляр класса и в параметрах указываем (номер пина кнопки 2, количество секунд нажатия кнопки) - для обнаружения длительного нажатия
SBB_CLICK Buttom3(6,1);             // создаём экземпляр класса и в параметрах указываем (номер пина кнопки 3, количество секунд нажатия кнопки) - для обнаружения длительного нажатия

float vario_down = -1.1;            // Установка порогового значения СНИЖЕНИЯ для запуска пищалки         - МОЖНО МЕНЯТЬ только в паре с TrDNcount, значения брать из массива treshDN!!!
float vario_up = 0.1;               // Установка порогового значения ПОДЪЁМА для запуска пищалки          - МОЖНО МЕНЯТЬ только в паре с TrUPcount, значения брать из массива treshUP!!!
float beep;                         // переменная для вывода звука при работе вариометра                  
float Beep_period;                  // переменная для вывода звука при работе вариометра                  
float vario = 0;                    // переменная вариометра                                              
float varioAvg = 0;                 // усреднённое значение вариометра для экрана
float avgAlt[31];                   // снимки высоты раз в секунду (хватает на 30 секунд)
byte avgIdx = 0;                    // позиция в кольцевом буфере
byte avgFill = 0;                   // сколько снимков уже накоплено
unsigned long avgTime = 0;          // время последнего снимка
byte AvgCount = 1;                  // выбранное время усреднения (индекс в avgSec)
const byte avgSec[5] = {5, 10, 15, 20, 30};   // МОЖНО МЕНЯТЬ!!! варианты времени усреднения, сек
float maxVario = 0;                 // переменная максимального значения вариометра                       
float minVario = 0;                 // переменная минимального значения вариометра                        
float maxVarioAvg = 0;              // максимальное усреднённое значение вариометра
float minVarioAvg = 0;              // минимальное усреднённое значение вариометра
float presSea = 1013.25;            // давление на уровне моря в hPa (DEFAULT=1013.25)   - МОЖНО МЕНЯТЬ!!!
float presTMP = 0;                  // переменная для работы процедуры ручного ввода давления
float tempo = millis();             // описание переменной tempo типа float и присвоение ей значения функции millis() - отсчитывает время от начала работы программы в миллисекундах
float N1 = 0;                       // переменная для работы усреднителя                                 
float N2 = 0;                       // переменная для работы усреднителя                                 
float N3 = 0;                       // переменная для работы усреднителя                                 
float D1 = 0;                       // переменная для работы усреднителя                                 
float D2 = 0;                       // переменная для работы усреднителя                                 

float alt[51];                                                          // Массив alt переменных типа float (для работы усреднителя), начальная размерность 51, моя 41
float tim[51];                                                          // Массив tim переменных типа float (для работы усреднителя), начальная размерность 51, моя 41 
float treshUP[5] = {0.1, 0.4, 0.5, 0.6, 0.7};                           // Массив, содержащий значения срабатывания вариометра при подъёме                       - МОЖНО МЕНЯТЬ!!! зависимые переменные: vario_up и TrUPcount
float treshDN[5] = {-0.5, -0.7, -0.9, -1.1, -1.3};                      // Массив, содержащий значения срабатывания вариометра при спуске                        - МОЖНО МЕНЯТЬ!!! зависимые переменные: vario_down и TrDNcount


int altTmp = 0;                     // переменная для вычисления высоты при обнулении КНОПКОЙ 1 (высота = высота с датчика - сохранённая высота в данной точке (altTmp))
int altTmpAdd = 0;                  // переменная для вычисления дополнительной высоты при обнулении КНОПКОЙ 1 (высота = высота с датчика - сохранённая высота в данной точке (altTmpAdd))
int altSEATmp = 0;                  // переменная для вычисления высоты, необходимой при обнулении максимального значения в ГЛАВНОМ ЭКРАНЕ 2
int altSEATmpZero = 0;              // переменная, необходимая при расчёте высоты над морем
int altitude = 0;                   // переменная высоты
int altitudeAdd = 0;                // переменная дополнительной высоты
int altitudeSEA = 0;                // переменная высоты над уровнем моря
int altdif = 0;                     // разница между высотой над стартом и высотой по давлению
int maxAltSea = 0;                  // максимальная высота над морем
int maxAltStt = 0;                  // максимальная высота над стартом
int maxAltAdd = 0;                  // максимальная высота дополнительная

char s0 = '7';                      // символ в значении давления по морю (используется для работы при вводе значения)
char s1 = '6';                      // символ в значении давления по морю (используется для работы при вводе значения)
char s2 = '0';                      // символ в значении давления по морю (используется для работы при вводе значения)
char s3 = '0';                      // символ в значении давления по морю (используется для работы при вводе значения)
char s4 = '0';                      // символ в значении давления по морю (используется для работы при вводе значения)

String presSTR;                     // строка используется в процедуре ручного ввода давления
char t0 = '0';                      // символы в значении высоты (для меню установки высоты)
char t1 = '0';                      //
char t2 = '0';                      //
char t3 = '0';                      //
String altSTR;                      // строка редактируемой высоты
int altSet = 0;                     // введённая высота в метрах
boolean altChange = false;          // включение/выключение режима ввода высоты
boolean altMode = false;            // false - задаём давление в hPa, true - задаём высоту в метрах
boolean B3Alt = false;              // нажатие кнопки 3 в режиме ввода высоты

boolean Stmp = false;               // переменная для вывода символа "B" (НЕОБЯЗАТЕЛЬНАЯ) 
boolean longpress = false;          // переменная для проверки на долгое нажатие кнопки
boolean beep_ON = true;             // переменная для включения/выключения звука вариометра
boolean button_beep_ON = true;      // переменная для включения/выключения звука клавиш                                   - МОЖНО МЕНЯТЬ!!!
boolean LED_ON = false;             // переменная для включения/выключения светодиодной индикации                         - МОЖНО МЕНЯТЬ!!!
boolean gLEDtmp = false;            // переменная для включения/выключения зеленого светодиода
boolean rLEDtmp = false;            // переменная для включения/выключения красного светодиода
boolean MainMenu = false;           // переменная проверки входа в меню
boolean askSave = false;            // показан ли экран вопроса "сохранить настройки?"
boolean saveChoice = true;          // выбранный ответ на этот вопрос (YES/NO)
boolean presChange = false;         // включение/выключение режима ввода давления
boolean B3Pres = false;             // увеличение символа в строке давления на единицу, если есть факт нажатия кнопки
boolean TresUPchange = false;       // флаг для изменения значения порога срабатывания на подъёме при нажатии клавиши 3
boolean TresDNchange = false;       // флаг для изменения значения порога срабатывания на сливе при нажатии клавиши 3
boolean AlarmFlag = false;          // флаг для работы сигнализации при падении
boolean Alarm = false;              // флаг ВКЛ/ВЫКЛ сигнализации при падении
boolean AfterAlarmFlag = false;     // флаг для затирания оставшихся после тревоги символов
boolean AltDifTmpFlag = false;      // флаг проверки был ли сброс высоты, или нет (единый для всех высот, но в коде выбирается проверка по значению cnt)
boolean Thermal_ON = false;         // флаг проверки включения режима Thermal для вариометра

static char outtemp[5];             // массив символов для отображения температуры с округлением до десятых
static char outvario[5];            // массив символов для отображения данных вариометра с округлением до десятых
static char outalt[5];              // массив символов для отображения высоты
static char outTrUP[4];             // массив символов для отображения корректируемого порога срабатывания на подъёме (в МЕНЮ)
static char outTrDN[4];             // массив символов для отображения корректируемого порога срабатывания на сливе (в МЕНЮ)
static char outtime[4];             // массив символов для отображения времени

unsigned long timeZero = 0;         // переменная для обнуления таймера (ИМЕННО unsigned long!!!)

byte TrUPcount = 0;                 // счётчик для выбора значения порога срабатывания на подъёме из массива возможных вариантов    - МОЖНО МЕНЯТЬ только в паре с vario_up, номер ячейки брать из массива treshUP!!!
byte TrDNcount = 3;                 // счётчик для выбора значения порога срабатывания на сливе из массива возможных вариантов      - МОЖНО МЕНЯТЬ только в паре с vario_down, номер ячейки брать из массива treshDN!!!
byte cnt = 4;                       // счётчик нажатий кнопки 1
byte volCount = 3;                  // громкость пищалки: 0-OFF, 1-LOW, 2-MED, 3-MAX
const byte volDuty[4] = {0, 2, 6, 15};  // МОЖНО МЕНЯТЬ!!! скважность ШИМ для каждой громкости, %
unsigned long toneOff = 0;          // время, когда нужно выключить звук
byte menuCount = 0;                 // счётчик для выбора режима меню
byte windowCount = 0;               // счётчик для выбора окна
byte symCount = 0;                  // номер вводимого символа (ввод давления)
byte symbolINT = 0;                 // символ integer для изменения в процедуре ввода давления
byte samples = 40;                  // исходное 40, моё 30 | что бы увеличить скорость реакции на изменение высоты, значение переменных samples и maxsamples надо уменьшать, но
byte maxsamples = 50;               // исходное 50, моё 40 | в таком случае повысится количество ложных срабатываний

byte degree[8]                      // кодируем символ градуса 
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
byte arrow_up[8]                    // кодируем символ стрелки вверх 
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
byte arrow_down[8]                  // кодируем символ стрелки вниз 
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
byte A_1[8]                         // кодируем символ ВЫСОТА НАД МОРЕМ
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
byte A_2[8]                         // кодируем символ ВЫСОТА НАД СТАРТОМ
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
byte A_3[8]                         // кодируем символ ВЫСОТА 3
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
byte arrow_e_up[8]                  // кодируем символ пустой стрелки вверх
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
byte arrow_e_down[8]                // кодируем символ пустой стрелки вниз
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

///// НАЧАЛЬНЫЕ УСТАНОВКИ / INITIAL SETUP /////
///// СОХРАНЕНИЕ НАСТРОЕК В ПАМЯТЬ / SAVED SETTINGS (EEPROM) /////
struct SavedSettings {
  uint16_t magic;
  byte TrUP, TrDN, vol, avg;
  byte led, btnBeep, thermal;
  float presSea;
};
#define SETTINGS_MAGIC 0xA55C       // если изменить это число, сохранённые настройки сбросятся

void applySettings()
{
  vario_up = treshUP[TrUPcount];
  vario_down = treshDN[TrDNcount];
}

void loadSettings()
{
  SavedSettings s;
  EEPROM.get(0, s);
  if (s.magic == SETTINGS_MAGIC && s.TrUP < 5 && s.TrDN < 5 && s.vol < 4 && s.avg < 5 && s.presSea > 300.0 && s.presSea < 1200.0)
    {
      TrUPcount = s.TrUP; TrDNcount = s.TrDN; volCount = s.vol; AvgCount = s.avg;
      LED_ON = s.led; button_beep_ON = s.btnBeep; Thermal_ON = s.thermal;
      presSea = s.presSea;
    }
}

void saveSettings()
{
  SavedSettings s;
  s.magic = SETTINGS_MAGIC;
  s.TrUP = TrUPcount; s.TrDN = TrDNcount; s.vol = volCount; s.avg = AvgCount;
  s.led = LED_ON; s.btnBeep = button_beep_ON; s.thermal = Thermal_ON;
  s.presSea = presSea;
  EEPROM.put(0, s);                 // записываются только изменившиеся байты
}

///// СНИМОК НАСТРОЕК ПРИ ВХОДЕ В МЕНЮ / SETTINGS BACKUP ON MENU ENTRY /////
byte bkTrUP, bkTrDN, bkVol, bkAvg;  // копии настроек на момент входа в меню
boolean bkLED, bkBtn, bkTherm;      //
float bkPres;                       //

void backupSettings()               // запоминаем настройки при входе в меню
{
  bkTrUP = TrUPcount; bkTrDN = TrDNcount; bkVol = volCount; bkAvg = AvgCount;
  bkLED = LED_ON; bkBtn = button_beep_ON; bkTherm = Thermal_ON;
  bkPres = presSea;
}

void restoreSettings()              // возвращаем настройки, какими они были при входе в меню
{
  TrUPcount = bkTrUP; TrDNcount = bkTrDN; volCount = bkVol; AvgCount = bkAvg;
  LED_ON = bkLED; button_beep_ON = bkBtn; Thermal_ON = bkTherm;
  presSea = bkPres;
  applySettings();
  avgFill = 0;                      // усреднение начинаем заново
}

///// ВЫХОД ИЗ МЕНЮ С ОТВЕТОМ НА ВОПРОС О СОХРАНЕНИИ / LEAVE MENU /////
void exitMenu()
{
  askSave = false;                    // убираем экран вопроса
  altChange = false;                  // выключаем режим ввода высоты
  MainMenu = false;                   // закрываем меню
  menuCount = 0;                      // обнуляем счётчик режимов меню
  symCount = 0;                       // обнуляем счётчик символов
  if (presChange == true)             // если выходим в режиме редактирования давления...
    {
      presChange = false;             // ... выключаем режим
      presTMP = presSea;              // ... возвращаем прежнее значение
      presSTR = presToStr(presTMP);   //
    }
  if (saveChoice == true) {saveSettings();}   // YES - сохраняем настройки в память
  else                                        // NO - возвращаем настройки, какими они были при входе в меню
    {
      restoreSettings();              //
    }
  presTMP = presSea;                  //
  presSTR = presToStr(presTMP);       //
  beep_ON = true;                     // включаем звуки вариометра
  lcd.clear();                        // очищаем экран
}

String presToStr(float v)           // давление в виде строки "1013.2" (4 цифры + 1 после точки)
{
  char buf[10];
  dtostrf(v, 6, 1, buf);
  for (byte i = 0; buf[i] == ' '; i++) {buf[i] = '0';}
  return String(buf);
}

void setup() 
{ 
  //Serial.begin(9600);               // установка порта на скорость 9600 бит/сек

  loadSettings();                   // читаем сохранённые настройки из энергонезависимой памяти
  presTMP = presSea;                // временная переменная давления (для расчётов) = дефолтное давление (presSea используется в формуле расчёта высоты)
  presSTR = presToStr(presTMP);        // преобразование давления из float в String
  applySettings();                  // применяем пороги из сохранённых настроек (vario_up, vario_down)
  backupSettings();                 // первый снимок настроек (на случай ответа NO до первого входа в меню)
  
  pinMode(p, OUTPUT);               // установка пина 3 в output (это пин пищалки)
  pinMode(gLED, OUTPUT);            // установка пина 7 в output (это пин зеленого светодиода)
  pinMode(rLED, OUTPUT);            // установка пина 8 в output (это пин красного светодиода)
  
  pinMode(switchPin1, INPUT);       // установка пина 4 в input (это пин кнопки)
  digitalWrite(switchPin1, LOW);    // отключаем внутреннюю подтяжку; используется внешний резистор 10 кОм
  b1.attach(switchPin1);            // объект Bounce будет слушать кнопку на пине switchPin1
  b1.interval(5);                   // устанавливаем время задержки в 5 мс

  pinMode(switchPin2, INPUT);       // установка пина 5 в input (это пин кнопки)
  digitalWrite(switchPin2, LOW);    // отключаем внутреннюю подтяжку; используется внешний резистор 10 кОм
  b2.attach(switchPin2);            // объект Bounce будет слушать кнопку на пине switchPin2
  b2.interval(5);                   // устанавливаем время задержки в 5 мс

  pinMode(switchPin3, INPUT);       // установка пина 6 в input (это пин кнопки)
  digitalWrite(switchPin3, LOW);    // отключаем внутреннюю подтяжку; используется внешний резистор 10 кОм
  b3.attach(switchPin3);            // объект Bounce будет слушать кнопку на пине switchPin3
  b3.interval(5);                   // устанавливаем время задержки в 5 мс
  
  lcd.init();                       // инициализация lcd 
  lcd.setCursor(0,0);               //
  lcd.print(" HELLO ERNESTO ");    // первая строка экрана приветствия
  lcd.setCursor(0,1);               //
  lcd.print("BARO SENSOR TEST");    // вторая строка экрана приветствия
  delay(1000);                      // задержка 1 секунда
    for (int i = 0; i<2; i++)             // чуть поморгаем диодами при старте. Типа с какой-то высшей целью, а на самом деле, это просто круто выглядит! =)
      {                                   //
        digitalWrite(gLED, HIGH);         // включаем зеленый светодиод при старте прибора
        digitalWrite(rLED, HIGH);         // включаем красный светодиод при старте прибора
        delay(50);                        //
        digitalWrite(gLED, LOW);          // вЫключаем зеленый светодиод при старте прибора
        digitalWrite(rLED, LOW);          // вЫключаем красный светодиод при старте прибора
        delay(100);                       //
      }                                   //
  lcd.clear();                      // очистка экрана
  lcd.setCursor(3,0);               //
  lcd.print("def. pres.");          // надпись "дефолтное давление"
  lcd.setCursor(5,1);               //
  lcd.print(String(presSea,1)+" hPa");   // выводим стартовое давление в hPa
  delay(1000);                      // задержка 2 секунды
  playTone((2500),70);                // двойной сигнал при запуске прибора  
  beepDelay(150);                       //
  playTone((2500),70);                //
  beepDelay(70);                   //
  lcd.clear();                      // очищаем экран перед отображением главного окна
  
  sensor.begin();                   // инициализация сенсора bmp180. в скобках можно указать начальную высоту в метрах 
  
  lcd.createChar(1, degree);        // создаём значок градуса 
  lcd.createChar(2, arrow_up);      // создаём значок стрелки вверх
  lcd.createChar(3, arrow_down);    // создаём значок стрелки вниз
  lcd.createChar(4, A_1);           // создаём значок A_1
  lcd.createChar(5, A_2);           // создаём значок A_2
  lcd.createChar(6, A_3);           // создаём значок A_3
  lcd.createChar(7, arrow_e_up);    // создаём значок пустой стрелки вверх
  lcd.createChar(0, arrow_e_down);  // создаём значок пустой стрелки вверх
} 

///// ПОДПРОГРАММА ВЫВОДА ДАННЫХ НА ЭКРАН / LCD OUTPUT /////
void lcd_out()                                                          // функция вывода значений на экран code by nuclear_XONIX (Ильюшенко Андрей, Iliushenko Andrei)
{                                                                       // 
if (Alarm == false)                                                     // если тревога выключена
{                                                                       //
if (AfterAlarmFlag == true)                                             // если взведён флаг очистки после тревоги, то
  {lcd.clear(); AfterAlarmFlag = false;}                                // очищаем экран и сбрасываем флаг

///// ГЛАВНОЕ ОКНО 1 / MAIN WINDOW 1 /////
if (windowCount == 0)                                                   // если выбран первый экран меню
{                                                                       //
  
///// ВЫВОД СИМВОЛА РАБОТЫ ЗВУКА ВАРИОМЕТРА "B" / SYMBOL "B" OUTPUT (for Vario) /////
  lcd.setCursor(5,0);                                                   //
    if (Stmp==true) {lcd.print("B");}                                   // вывод символа "B" (beep) при наличии звука пищалки (на случай, если динамик отключён)
    else {lcd.print(" ");}                                              // вывод символа " " при отсутствии звука пищалки (на случай, если динамик отключён)
      
///// ВЫВОД СИМВОЛА ДИНАМИКА "S" (SOUND) / SYMBOL "S" OUTPUT (for Speaker) /////
  lcd.setCursor(9,0);                                                   //
  if (beep_ON == true) {lcd.print("S");}                                // если звук включён, то рисуем символ SOUND ("S")
  else {lcd.print(" ");}                                                // иначе рисуем пустой символ (" ")

///// ВЫВОД ИНДИКАТОРА ВАРИОМЕТРА (стрелки) / VARIO INDICATION OUTPUT (Arrows) /////
    if (vario>0.1)                                                      // если значение вариометра положительное (>0.1 для компенсации погрешности работы датчика), то закрашиваем стрелку вверх
    {                                                                   //
      lcd.setCursor(0,0);                                               //
      lcd.write(byte(2));                                               //
    }                                                                   // 
    if (vario<-0.1)                                                     // если значение вариометра отрицательное (<-0.1 для компенсации погрешности работы датчика), то закрашиваем стрелку вниз
    {                                                                   //
      lcd.setCursor(0,1);                                               //
      lcd.write(byte(3));                                               //
    }                                                                   // 
    if ((vario>=-0.1) && (vario<=0.1))                                  // если значение вариометра нулевое (от -0.02 до 0 для компенсации погрешности работы датчика), то рисуем две пустых стрелки вверх и вниз
    {                                                                   //
      lcd.setCursor(0,0);                                               //
      lcd.write(byte(7));                                               //
      lcd.setCursor(0,1);                                               //
      lcd.write(byte(0));                                               //
    }                                                                   // 

///// ВЫВОД ЗНАЧЕНИЙ ВАРИОМЕТРА / VARIO OUTPUT /////
  dtostrf(varioAvg,4, 1, outvario);                                        // перевод значения переменной vario (float) в массив символов для возможности округления до десятых
  lcd.setCursor(1,0);                                                   //
  lcd.print(outvario);                                                  // вывод значений вариометра с округлением до десятых

///// ВЫВОД ТЕМПЕРАТУРЫ / TEMPERATURE OUTPUT (C) /////
  dtostrf(sensor.temp,5, 1, outtemp);                                   // перевод параметра сенсора sensor.temp в массив символов для возможности округления до десятых
  lcd.setCursor(10,1);                                                  //
  lcd.print(outtemp);                                                   // вывод значения температуры с округлением до десятых
  lcd.write(byte(1));                                                   // рисуем символ градуса при выводе значений температуры 

///// ВЫВОД ВРЕМЕНИ / TIME OUTPUT (Min) /////
  lcd.setCursor(11,0);                                                  //
  lcd.print("Tm");                                                      // печатаем Tm (Time minute)
  dtostrf((((millis()-timeZero)/1000)/60),3, 0, outtime);               // перевод значения отсчитанного времени в массив символов для возможности округления до целых
  lcd.print(outtime);                                                   // вывод значений времени с округлением до целых

///// ВЫВОД ВЫСОТЫ НАД УРОВНЕМ МОРЯ / ALT ABOVE SEA OUTPUT (m) /////
if (cnt==4)                                                             // ВЫСОТА НА УРОВНЕ МОРЯ
  {                                                                     //
  lcd.setCursor(2,1);                                                   //
  lcd.write(byte(cnt));                                                 // вывод символа высоты (cnt - счётчик, обновляемый при нажатии кнопки)
  dtostrf(float(altitudeSEA),4, 0, outalt);                             // перевод параметра сенсора sensor.temp в массив символов для возможности округления до десятых
  lcd.print(outalt);                                                    // вывод значения температуры с округлением до десятых
  lcd.print("m");                                                       //
  }                                                                     //

///// ВЫВОД ВЫСОТЫ НАД СТАРТОМ / ALT ABOVE TAKE OFF POINT OUTPUT (m) /////
if (cnt==5)                                                             // ВЫСОТА НА УРОВНЕ СТАРТА
  {                                                                     // 
  lcd.setCursor(2,1);                                                   //
  lcd.write(byte(cnt));                                                 // вывод символа высоты (cnt - счётчик, обновляемый при нажатии кнопки)
  dtostrf(float(altitude),4, 0, outalt);                                // перевод параметра сенсора sensor.temp в массив символов для возможности округления до десятых
  lcd.print(outalt);                                                    // вывод значения температуры с округлением до десятых
  lcd.print("m");                                                       //
  }                                                                     //

///// ВЫВОД ДОПОЛНИТЕЛЬНОЙ ВЫСОТЫ / ADDITIONAL ALT OUTPUT (m) /////
if (cnt==6)                                                             // ВЫСОТА ДОПОЛНИТЕЛЬНАЯ
  {                                                                     //
  lcd.setCursor(2,1);                                                   //
  lcd.write(byte(cnt));                                                 // вывод символа высоты (cnt - счётчик, обновляемый при нажатии кнопки)
  dtostrf(float(altitudeAdd),4, 0, outalt);                             // перевод параметра сенсора sensor.temp в массив символов для возможности округления до десятых
  lcd.print(outalt);                                                    // вывод значения температуры с округлением до десятых
  lcd.print("m");                                                       //
  }                                                                     //
}                                                                       //

///// ГЛАВНОЕ ОКНО 2 / MAIN WINDOW 2 (Max altitudes) /////
if (windowCount == 1)                                                                   // если выбран второй экран меню
  {                                                                                     //
    lcd.setCursor(0,0);                                                                 //
    lcd.print("Max Alt: "); lcd.write(byte(4)); lcd.print(" "+String(maxAltSea)+"m");   // рисуем заголовок и значение максимальной высоты над морем
    lcd.setCursor(0,1);                                                                 //
    lcd.write(byte(5)); lcd.print(" "+String(maxAltStt)+"m");                           // рисуем значение максимальной высоты над стартом
    lcd.setCursor(9,1);                                                                 //
    lcd.write(byte(6)); lcd.print(" "+String(maxAltAdd)+"m");                           // рисуем значение максимальной дополнительной высоты
  }                                                                                     //

///// ГЛАВНОЕ ОКНО 3 / MAIN WINDOW 3 (Max/Min vario) /////
if (windowCount == 2)                                                   // если выбран третий экран меню
  {                                                                     //
    lcd.setCursor(0,0);                                                 //
    lcd.print("Max Var:   "); lcd.print(maxVario);                      // печатаем максимальное значение вариометра
    lcd.setCursor(0,1);                                                 //
    lcd.print("Min Var:   "); lcd.print(minVario);                      // печатаем минимальное значение вариометра
  }                                                                     //

///// ЭКРАННОЕ ОКНО 4 / MAIN WINDOW 4 (Max/Min averaged vario) /////
if (windowCount == 3)                                                   // если выбрано четвёртое окно...
  {                                                                     //
    lcd.setCursor(0,0);                                                 //
    lcd.print("Max Avg:   "); lcd.print(maxVarioAvg);                   // выводим максимальное усреднённое значение
    lcd.setCursor(0,1);                                                 //
    lcd.print("Min Avg:   "); lcd.print(minVarioAvg);                   // выводим минимальное усреднённое значение
  }                                                                     //
}                                                                       //
}                                                                       // конец вывода на экран

///// ЗВУК С РЕГУЛИРОВКОЙ ГРОМКОСТИ / TONE WITH VOLUME CONTROL /////
///// Пищалка на пине 3 (OC2B). Частоту задаёт таймер 2, громкость - скважность ШИМ.
void stopTone()                                                         // выключаем звук
{
  TCCR2A = 0;                                                           //
  TCCR2B = 0;                                                           //
  digitalWrite(p, LOW);                                                 //
  toneOff = 0;                                                          //
}

void playTone(unsigned int freq, unsigned int dur)                      // звук заданной частоты и длительности
{
  if (volCount == 0) {stopTone(); return;}                              // громкость OFF - молчим
  if (freq < 250) {freq = 250;}                                         // ниже таймер 2 не может
  if (freq > 5000) {freq = 5000;}                                       //
  byte top = (62500UL / freq) - 1;                                      // предделитель 256
  OCR2A = top;                                                          // частота
  OCR2B = (unsigned int)top * volDuty[volCount] / 100;                  // громкость
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);                       // быстрый ШИМ на OC2B
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21);                          // TOP = OCR2A, предделитель 256
  toneOff = millis() + dur;                                             // когда выключить
}

void toneService()                                                      // вызывается в каждом цикле - выключает звук вовремя
{
  if ((toneOff != 0) && (millis() >= toneOff)) {stopTone();}            //
}

void beepDelay(unsigned int ms)                                         // пауза, во время которой звук выключается вовремя
{
  unsigned long t = millis() + ms;                                      //
  while (millis() < t) {toneService();}                                 //
}

///// СИГНАЛ НАЖАТИЯ КНОПКИ / BUTTON BEEP /////
void button_beep()                                                      // СИГНАЛ НАЖАТИЯ КНОПКИ
{                                                                       //
  if (button_beep_ON == true) {playTone(1800,30); beepDelay(30);}                        // пищим только если в настройках включен звук клавиш
}                                                                       //

///// ДЛИННЫЙ БИП / LONG BEEP /////
void long_beep()                                                        // ДЛИННЫЙ БИП
{                                                                       //
  if (button_beep_ON == true) {playTone(1800,200); beepDelay(200);}                       // пищим только если в настройках включен звук клавиш
}                                                                       //

///// ТРОЙНОЙ БИП / TRIPLE BEEP /////
void triple_beep()                                                      // ТРОЙНОЙ БИП
{                                                                       //
  if (button_beep_ON == true)                                           // пищим только если в настройках включен звук клавиш
  {                                                                     //
    playTone(1800,40);                                                    //
    beepDelay(80);                                                          //
    playTone(1800,40);                                                    //
    beepDelay(80);                                                          //
    playTone(1800,40);                                                    // 
  }                                                                     //
}                                                                       //

///// КОРОТКИЙ-КОРОТКИЙ-ДЛИННЫЙ БИП / SHORT-SHORT-LONG BEEP /////
void SSL_beep()                                                         // КОРОТКИЙ-КОРОТКИЙ-ДЛИННЫЙ БИП
{                                                                       //
  if (button_beep_ON == true)                                           // пищим только если в настройках включен звук клавиш
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

///// ОСНОВНОЕ МЕНЮ / MAIN MENU /////
void Menu()                                                                       //
{                                                                                 //
  if (askSave == true)                                                            // экран вопроса о сохранении настроек
    {
      lcd.setCursor(0,0);                                                           //
      lcd.print("Save settings?  ");                                                //
      lcd.setCursor(0,1);                                                           //
      if (saveChoice == true) {lcd.print("  > YES     no  ");}                      // выбран ответ YES
      else                    {lcd.print("    yes   > NO  ");}                      // выбран ответ NO
      return;                                                                       //
    }

  lcd.setCursor(0,0);                                                             //
  lcd.print("MENU "+String(menuCount+1)+"   ");                                   // выводим надпись MAIN MENU и номер окна меню на верхней строчке
    if (menuCount == 0) {SeaPresChange();}                                        // 1 экран меню - давление по морю / высота
    else if (menuCount == 1) {TresholdUP();}                                      // 2 экран меню - порог срабатывания на подъёме
    else if (menuCount == 2) {TresholdDN();}                                      // 3 экран меню - порог срабатывания на снижении
    else if (menuCount == 3) {LED_light();}                                       // 4 экран меню - ВКЛ/ВЫКЛ светодиодов
    else if (menuCount == 4) {ButtonBeep();}                                      // 5 экран меню - ВКЛ/ВЫКЛ звука клавиш
    else if (menuCount == 5) {VolumeSet();}                                       // 6 экран меню - громкость пищалки
    else if (menuCount == 6) {VarioMode();}                                       // 7 экран меню - режим работы вариометра
    else if (menuCount == 7) {AvgTime();}                                         // 8 экран меню - время усреднения показаний
}                                                                                 //

///// УСТАНОВКА ВЫСОТЫ, ИЗ КОТОРОЙ СЧИТАЕТСЯ ДАВЛЕНИЕ / SET ALTITUDE, PRESSURE IS CALCULATED /////
char nextDigit(char c)              // следующая цифра (после 9 снова 0)
{
  byte v = int(c);                  //
  v++;                              //
  if (v == 58) {v = 48;}            //
  return char(v);                   //
}

void altToDigits(int v)             // раскладываем высоту по четырём символам
{
  if (v < 0) {v = 0;}               //
  if (v > 9999) {v = 9999;}         //
  t0 = char(48+(v/1000)%10);        //
  t1 = char(48+(v/100)%10);         //
  t2 = char(48+(v/10)%10);          //
  t3 = char(48+v%10);               //
}

void presToDigits(float v)          // раскладываем давление по пяти символам
{
  presSTR = presToStr(v);           //
  s0 = presSTR[0];                  //
  s1 = presSTR[1];                  //
  s2 = presSTR[2];                  //
  s3 = presSTR[3];                  //
  s4 = presSTR[5];                  //
}


///// ПРОЦЕДУРА ИЗМЕНЕНИЯ ЗНАЧЕНИЯ ДАВЛЕНИЯ ПО МОРЮ / SEA PRESSURE VALUE CHANGE PROCEDURE /////
void SeaPresChange()                                                              // объединённая страница: давление по морю / высота
{
  if (altMode == false)                                                           // ===== РЕЖИМ "ДАВЛЕНИЕ" =====
    {
      if (presChange == true)                                                     // если идёт ввод...
        {
          if (B3Pres == true)                                                     // ... и нажата кнопка 3, меняем выбранную цифру
            {
              if (symCount == 0) {s0 = nextDigit(s0);}                            //
              if (symCount == 1) {s1 = nextDigit(s1);}                            //
              if (symCount == 2) {s2 = nextDigit(s2);}                            //
              if (symCount == 3) {s3 = nextDigit(s3);}                            //
              if (symCount == 5) {s4 = nextDigit(s4);}                            //
              B3Pres = false;                                                     //
            }
          presSTR = (String)s0+s1+s2+s3+"."+s4;                                   // собираем строку давления
          presTMP = presSTR.toFloat();                                            //
          presSea = presTMP;                                                      // сразу применяем - высота пересчитывается мгновенно
          lcd.setCursor(10,0);                                                    //
          lcd.print("     ");                                                     // стираем старый курсор
          lcd.setCursor((10+symCount),0);                                         //
          lcd.print("v");                                                         // рисуем курсор над выбранной цифрой
        }
      else                                                                        // если ввод выключен - показываем текущее давление
        {
          presToDigits(presSea);                                                  // обновляем цифры из presSea (могли поменять через высоту)
          presTMP = presSea;                                                      //
          lcd.setCursor(10,0);                                                    //
          lcd.print("      ");                                                    // стираем курсор
        }
      lcd.setCursor(0,1);                                                         //
      lcd.print("Sea pres. ");                                                    // рисуем строку "Sea pres."
      lcd.setCursor(10,1);                                                        //
      lcd.print(presSTR);                                                         // выводим давление в hPa
    }
  else                                                                            // ===== РЕЖИМ "ВЫСОТА" =====
    {
      if (altChange == true)                                                      // если идёт ввод...
        {
          if (B3Alt == true)                                                      // ... и нажата кнопка 3, меняем выбранную цифру
            {
              if (symCount == 0) {t0 = nextDigit(t0);}                            //
              if (symCount == 1) {t1 = nextDigit(t1);}                            //
              if (symCount == 2) {t2 = nextDigit(t2);}                            //
              if (symCount == 3) {t3 = nextDigit(t3);}                            //
              B3Alt = false;                                                      //
            }
          altSTR = (String)t0+t1+t2+t3;                                           // собираем строку высоты
          altSet = altSTR.toInt();                                                //
          presSea = (sensor.pres*MMHG_TO_HPA)*exp(float(altSet)/(29.21*(sensor.temp+273.15)));   // сразу считаем давление из высоты
          presToDigits(presSea);                                                  // и обновляем цифры давления
          presTMP = presSea;                                                      //
          lcd.setCursor(11,0);                                                    //
          lcd.print("    ");                                                      // стираем старый курсор
          lcd.setCursor((11+symCount),0);                                         //
          lcd.print("v");                                                         // рисуем курсор над выбранной цифрой
        }
      else                                                                        // если ввод выключен - показываем текущую высоту
        {
          altToDigits(altitudeSEA);                                               //
          altSTR = (String)t0+t1+t2+t3;                                           //
          lcd.setCursor(11,0);                                                    //
          lcd.print("     ");                                                     // стираем курсор
        }
      lcd.setCursor(0,1);                                                         //
      lcd.print("Set alt.   ");                                                   // рисуем строку "Set alt."
      lcd.setCursor(11,1);                                                        //
      lcd.print(altSTR+"m");                                                      // выводим высоту в метрах
    }
}

///// ПРОЦЕДУРА ВЫБОРА ПОРОГА НА ПОДЪЁМЕ / SETTING TRESHOLD FOR CLIMB /////
void TresholdUP()                                                                 //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print("Tr.hold UP ");                                                       // рисуем строку "Tr.hold UP "
  lcd.print("     ");                                                             // затираем старые символы
    if (TresUPchange == true)                                                     // если был факт нажатия КНОПКИ 3, то...
      {                                                                           //
        vario_up = treshUP[TrUPcount];                                            // присваиваем переменной vario_up (используется в при выводе звука на подъёме) новое значение из массива возможных значений
        TresUPchange = false;                                                     // убираем факт нажатия КНОПКИ 3
      }                                                                           // 
  lcd.setCursor(12,1);                                                            // 
  dtostrf(vario_up, 4, 1, outTrUP);                                               // переводим значение vario_up из float в массив символов для вывода на экран
  lcd.print(String(outTrUP));                                                     // выводим на экран значение порога срабатывания на подъёме
}                                                                                 //

///// ПРОЦЕДУРА ВЫБОРА ПОРОГА НА СЛИВЕ / SETTING TRESHOLD FOR DESCENT /////
void TresholdDN()                                                                 // 
{                                                                                 // 
  lcd.setCursor(0,1);                                                             // 
  lcd.print("Tr.hold DN ");                                                       // рисуем строку "Tr.hold DN "
  lcd.print("     ");                                                             // затираем старые символы
    if (TresDNchange == true)                                                     // если был факт нажатия КНОПКИ 3, то...
      {                                                                           //
        vario_down = treshDN[TrDNcount];                                          // присваиваем переменной vario_down (используется в при выводе звука на сливе) новое значение из массива возможных значений
        TresDNchange = false;                                                     // убираем факт нажатия КНОПКИ 3
      }                                                                           // 
  lcd.setCursor(12,1);                                                            // 
  dtostrf(vario_down, 4, 1, outTrDN);                                             // переводим значение vario_down из float в массив символов для вывода на экран
  lcd.print(String(outTrDN));                                                     // выводим на экран значение порога срабатывания на сливе
}                                                                                 //

///// ВКЛ/ВЫКЛ СВЕТОДИОДОВ / LED SETTING ON/OFF /////
void LED_light()                                                                  //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print("Light      ");                                                       // рисуем строку "Light"
  lcd.setCursor(13,1);                                                            //
    if (LED_ON == true) {lcd.print("ON ");}                                       // если индикация включена, то рисуем ON
    else {lcd.print("OFF");}                                                      // если выключена, то рисуем OFF
}                                                                                 //

///// ПОДПРОГРАММА ВЫБОРА РЕЖИМА ВАРИОМЕТРА / VARIO MODE SET /////
///// ВРЕМЯ УСРЕДНЕНИЯ ПОКАЗАНИЙ / AVERAGING TIME /////
void AvgTime()                                                                    //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print("Average    ");                                                       // рисуем строку "Average"
  lcd.setCursor(12,1);                                                            //
  if (avgSec[AvgCount] < 10) {lcd.print("  ");}                                   //
  else {lcd.print(" ");}                                                          //
  lcd.print(avgSec[AvgCount]);                                                    // выводим время усреднения
  lcd.print("s");                                                                 //
}                                                                                 //

///// ГРОМКОСТЬ ПИЩАЛКИ / BUZZER VOLUME /////
void VolumeSet()                                                                  //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print("Volume     ");                                                       // рисуем строку "Volume"
  lcd.setCursor(12,1);                                                            //
  if (volCount == 0) {lcd.print(" OFF");}                                         //
  else if (volCount == 1) {lcd.print(" LOW");}                                    //
  else if (volCount == 2) {lcd.print(" MED");}                                    //
  else {lcd.print(" MAX");}                                                       //
}                                                                                 //

///// РЕЖИМ РАБОТЫ ВАРИОМЕТРА / VARIO MODE SET /////
void VarioMode()                                                                  //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print("Vario mode ");                                                       // рисуем строку "Vario mode"
  lcd.setCursor(13,1);                                                            //
    if (Thermal_ON == true)                                                       // если выбран режим Thermal, то...
      {                                                                           //
        lcd.print("Trm");                                                         // рисуем "Trm"
        samples = 30;                                                             // уменьшаем количество сэмплов (скорость обработки увеличивается, но возрастает количество ложных срабатываний)
        maxsamples = 40;                                                          // уменьшаем количество сэмплов
        tempo = millis();                                                         // описание переменной tempo типа float и присвоение ей значения функции millis() - отсчитывает время от начала работы программы в миллисекундах 
        N1 = 0;                                                                   // переменная для работы усреднителя
        N2 = 0;                                                                   // переменная для работы усреднителя
        N3 = 0;                                                                   // переменная для работы усреднителя
        D1 = 0;                                                                   // переменная для работы усреднителя
        D2 = 0;                                                                   // переменная для работы усреднителя
        vario = 0;                                                                // обнуляем вариометр перед расчётом
      }                                                                           //
    else                                                                          // если выбран режим Standard, то...
      {                                                                           //
        lcd.print("Std");                                                         // рисуем "Std"
        samples = 40;                                                             // увеличиваем количество сэмплов (скорость обработки увеличивается, но возрастает количество ложных срабатываний)
        maxsamples = 50;                                                          // увеличиваем количество сэмплов
        tempo = millis();                                                         // описание переменной tempo типа float и присвоение ей значения функции millis() - отсчитывает время от начала работы программы в миллисекундах 
        N1 = 0;                                                                   // переменная для работы усреднителя
        N2 = 0;                                                                   // переменная для работы усреднителя
        N3 = 0;                                                                   // переменная для работы усреднителя
        D1 = 0;                                                                   // переменная для работы усреднителя
        D2 = 0;                                                                   // переменная для работы усреднителя
        vario = 0;                                                                // обнуляем вариометр перед расчётом
      }                                                                           // 
}                                                                                 //

///// ВКЛ/ВЫКЛ ЗВУКА КЛАВИШ / BUTTON BEEP ON/OFF /////
void ButtonBeep()                                                                 //
{                                                                                 //
  lcd.setCursor(0,1);                                                             //
  lcd.print("But. sound ");                                                       // рисуем строку "Button sound"
  lcd.setCursor(13,1);                                                            //
  if (button_beep_ON == true) {lcd.print("ON ");}                                 // если звук включен, то на экране рисуем значение ON
  else {lcd.print("OFF");}                                                        // если выключен, то рисуем значение OFF
}                                                                                 //

///// ГЛАВНЫЙ ЦИКЛ / MAIN CYCLE /////
void loop()                                                                               //
{                                                                                         //
toneService();                                                          // выключаем звук, когда пришло время
if (sensor.read(3))                                                                       // проверка на наличие данных с датчика 
{                                                                                         // чтение параметров с датчика bmp180. в скобках могут быть значения чувствительности от 0 до 3. 3 - максимум 
if (vario>-0.3) {Alarm = false;}                                                          // проверка ВКЛ/ВЫКЛ сигнала тревоги при падении

///// ОБРАБОТКА СОБЫТИЙ НА КНОПКЕ 1 / BUTTON 1 SHORT PRESS /////
if ((b1.update() == 1) && (b1.read() == LOW) && (b1.fell() == 1) && (longpress == false))   // если зарегистрировано событие и КНОПКА 1 нажата и не было зарегистрировано длительное нажатие (короче, это короткий клик)
  {                                                                                       //
    button_beep();                                                                        // сигнал при нажатии клавиши
    if (MainMenu == false)                                                                // ... и если мы не в МЕНЮ ...
      {                                                                                   //
        MainMenu = true;                                                                  // то включается режим меню
        backupSettings();                                                         // запоминаем настройки, что бы ответ NO мог их вернуть
          s0 = presSTR[0];                                                                // присваиваем строке s0 первый символ в значении давления
          s1 = presSTR[1];                                                                // присваиваем строке s1 второй символ в значении давления
          s2 = presSTR[2];                                                                // присваиваем строке s2 третий символ в значении давления
          s3 = presSTR[3];                                                                // присваиваем строке s3 четвертый символ в значении давления
          s4 = presSTR[5];                                                                // присваиваем строке s4 пятый символ в значении давления
        beep_ON = false;                                                                  // выключаем флаг beep_ON (то есть, в МЕНЮ не работает звук вариометра)
        lcd.clear();                                                                      // очистка экрана при нажатии кнопки
      }                                                                                   //
    else                                                                                  // ... если мы уже в меню
      {
        if (askSave == true)                                                              // если показан вопрос о сохранении...
          {
            exitMenu();                                                                   // ... выполняем ответ и выходим из меню
          }
        else if ((presChange == false) && (altChange == false))                           // иначе листаем страницы меню НАЗАД
          {
            if (menuCount == 0) {menuCount = 8;}                                          // с первой страницы переходим на последнюю
            menuCount -= 1;                                                               //
            lcd.clear();                                                                  //
          }
      }
  }                                                                                       //

///// ДЛИННОЕ НАЖАТИЕ КНОПКИ 1 / BUTTON 1 LONG PRESS /////
if (Buttom1.click())                                                                      // если кнопка 1 зажата больше 1 секунды, то... (это длинный клик)
  {                                                                                       //
    long_beep();                                                                          // длинный бип
    if (MainMenu == false)                                                                // ... и если мы не в МЕНЮ ...
      {                                                                                   //
        lcd.setCursor(13,0);                                                              // ОБНУЛЯЕМ ВРЕМЯ по длительному нажатию КЛАВИШИ 1
        lcd.print("   ");                                                                 // затираем старое значение времени
        timeZero = millis();                                                              // присвоение значения переменной для обнуления времени      
      }                                                                                   // 
    else                                                                                  // ... если мы в меню
      {
        if (askSave == false)                                                             // спрашиваем, сохранять ли настройки
          {
            askSave = true;                                                               // включаем экран вопроса
            saveChoice = true;                                                            // по умолчанию - YES
            lcd.clear();                                                                  //
          }
        else                                                                              // если вопрос уже показан - выполняем ответ
          {
            exitMenu();                                                                   //
          }
      }
    longpress = true;                                                                     // переменной присваивается true, т.е. факт длительного нажатия
  }                                                                                       //
if (b1.fell() == 1) {longpress = false;}                                                  // убираем факт длительного нажатия при отпускании книпки
Buttom1.release();                                                                        // обязательно надо прописывать после выполнения click(), иначе выполнится только раз

///// ОБРАБОТКА СОБЫТИЙ НА КНОПКЕ 2 / BUTTON 2 SHORT PRESS /////
if ((b2.update() == 1) && (b2.read() == LOW) && (b2.fell() == 1) && (longpress == false))   // если зарегистрировано событие и КНОПКА 2 нажата и не было зарегистрировано длительное нажатие (короткий клик)
  {                                                                                       //
    button_beep();                                                                        // сигнал при нажатии клавиши
    lcd.clear();                                                                          // очистка экрана при нажатии кнопки
    if (MainMenu == false)                                                                // ... и если мы не в МЕНЮ ...
      {                                                                                   // 
        if (windowCount == 0)                                                             // если мы в первом главном экране
          {                                                                               //
            cnt+=1;                                                                       // увеличиваем значение счётчика на единицу
            if (cnt==7)                                                                   // если значение счётчика дошло до 7, то...
              {                                                                           // ... присваиваем ему значение 4
                cnt=4;                                                                    // используются значения от 4 до 6, поскольку номера символов высоты идут от 4 до 6
              }                                                                           //
          }                                                                               //
      }                                                                                   //
    else if (askSave == true)                                                         // если показан вопрос о сохранении...
      {
        if (saveChoice == true) {saveChoice = false;}                                 // переключаем ответ YES/NO
        else {saveChoice = true;}                                                     //
      }
    else
      {                                                                                   //
        if (presChange == true)                                                           // если идёт ввод давления...
          {                                                                               //
            symCount += 1;                                                                // ... двигаем курсор по цифрам давления
            if (symCount == 4) {symCount += 1;}                                           // пропускаем позицию точки
            if (symCount == 6) {symCount = 0;}                                            // после последней цифры - снова первая
          }                                                                               //
        else if (altChange == true)                                                       // если идёт ввод высоты...
          {                                                                               //
            symCount += 1;                                                                // ... двигаем курсор по цифрам высоты
            if (symCount == 4) {symCount = 0;}                                            // после последней цифры - снова первая
          }                                                                               //
        else                                                                              // иначе листаем страницы меню
          {                                                                               //
            menuCount += 1;                                                               //
            if (menuCount == 8) {menuCount = 0;}                                         // после последней страницы - снова первая
          }                                                                               //
      }                                                                                   //
  }                                                                                       // 

///// ДЛИННОЕ НАЖАТИЕ КНОПКИ 2 / BUTTON 2 LONG PRESS /////
if (Buttom2.click())                                                                      // если кнопка 2 зажата больше 1 секунды, то... (длинный клик)
  {                                                                                       // 
    SSL_beep();                                                                           // вызов функции КОРОТКОГО-КОРОТКОГО-ДЛИННОГО ПИСКА
    if (MainMenu == false)                                                                // ... и если мы не в МЕНЮ ...
      {                                                                                   // ОБНУЛЕНИЕ ВЫСОТ ДЛИННЫМ НАЖАТИЕМ КНИПКИ 2
        if (cnt == 4)                                                                     // действие для ВЫСОТЫ по давлению на УРОВНЕ МОРЯ (обнуление высоты)
          {                                                                               //
            altSEATmp = altitudeSEA;                                                      // присвоение переменной altSEATmp значения расчётной высоты
            presSea = sensor.pres*MMHG_TO_HPA;   // давление в мм рт. ст. -> hPa
            presSTR = presToStr(presSea);                                                    // печатаем значение давления в переменную строки для отображения в МЕНЮ
            saveSettings();                                                               // сохраняем давление в память
          }                                                                               //
        if (cnt == 5)                                                                     // действие для ВЫСОТЫ на СТАРТЕ (обнуление высоты)
          {                                                                               //
            altTmp = sensor.alti;                                                         // присвоение переменной altTmp значения высоты с датчика
          }                                                                               //
        if (cnt == 6)                                                                     // действие для ДОПОЛНИТЕЛЬНОЙ ВЫСОТЫ (обнуление высоты)
          {                                                                               //
            altTmpAdd = sensor.alti;                                                      // присвоение переменной altTmpAdd значения высоты с датчика
          }                                                                               //
        AltDifTmpFlag = true;                                                             // переводим флаг, был сброс высоты
      }                                                                                   //
    else if (askSave == false)                                                                                  // ... если мы, всё-таки, в МЕНЮ ...
      {                                                                                   //
        if (menuCount == 0)                                                               // на первой странице - ввод давления или высоты
          {                                                                               //
            if (altMode == false)                                                         // --- режим "давление" ---
              {                                                                           //
                if (presChange == false)                                                  //
                  {                                                                       //
                    presChange = true;                                                    // включаем ввод давления
                    presToDigits(presSea);                                                //
                    presTMP = presSea;                                                    //
                    symCount = 0;                                                         //
                  }                                                                       //
                else                                                                      //
                  {                                                                       //
                    presChange = false;                                                   // заканчиваем ввод
                    presSea = presTMP;                                                    //
                    symCount = 0;                                                         //
                    altSEATmp = 0;                                                        //
                  }                                                                       //
              }                                                                           //
            else                                                                          // --- режим "высота" ---
              {                                                                           //
                if (altChange == false)                                                   //
                  {                                                                       //
                    altChange = true;                                                     // включаем ввод высоты
                    altToDigits(altitudeSEA);                                             // начинаем с текущей высоты над уровнем моря
                    symCount = 0;                                                         //
                  }                                                                       //
                else                                                                      //
                  {                                                                       //
                    altChange = false;                                                    // заканчиваем ввод
                    symCount = 0;                                                         //
                    altSEATmp = 0;                                                        //
                  }                                                                       //
              }                                                                           //
          }                                                                               //
      }                                                                                   //
    longpress = true;                                                                     // переменной присваивается true, т.е. факт длительного нажатия
    lcd.clear();                                                                          // очистка экрана
  }                                                                                       // 
if (b2.fell() == 1) {longpress = false;}                                                  // убираем факт длительного нажатия при отпускании книпки
Buttom2.release();                                                                        // обязательно надо прописывать после выполнения click(), иначе выполнится только раз

///// ОБРАБОТКА СОБЫТИЙ НА КНОПКЕ 3 / BUTTON 3 SHORT PRESS /////
if ((b3.update() == 1) && (b3.read() == LOW) && (b3.fell() == 1) && (longpress == false))   // если зарегистрировано событие и КНОПКА 3 нажата и не было зарегистрировано длительное нажатие (короткий клик)
  {                                                                                       //
    button_beep();                                                                        // сигнал при нажатии клавиши
    if (MainMenu == false)                                                                // ... если мы не в МЕНЮ, то...
      {                                                                                   // 
        windowCount++;                                                                    // ... переключаем главные экраны (в данной строке увеличиваем счётчик экрана)
        if (windowCount == 4)                                                             // если счётчик дошёл до трёх, то...
          {windowCount = 0;}                                                              // ... обнуляем
        lcd.clear();                                                                      // очищаем экран при каждом переключении
      }                                                                                   // 
    else if (askSave == true)                                                         // если показан вопрос о сохранении...
      {
        if (saveChoice == true) {saveChoice = false;}                                 // переключаем ответ YES/NO
        else {saveChoice = true;}                                                     //
      }
    else
      {                                                                                   // 
        if (presChange == true)                                                           // ... если также включен режим редактирования давления, то
          {                                                                               //
            B3Pres = true;                                                                // фиксируем факт нажатия КНОПКИ 3 (используется в подпрограмме SeaPresChange() для увеличения значения выбранного символа)
          }                                                                               //
        if (altChange == true)                                                             // ... если включён режим ввода высоты
          {                                                                               //
            B3Alt = true;                                                                 // отмечаем факт нажатия кнопки 3
          }                                                                               //
        if ((menuCount == 0) && (presChange == false) && (altChange == false))             // на первой странице вне режима ввода...
          {                                                                               //
            if (altMode == true) {altMode = false;}                                       // ... кнопка 3 переключает: давление <-> высота
            else {altMode = true;}                                                        //
            symCount = 0;                                                                 //
            lcd.clear();                                                                  //
          }                                                                               //
        if (menuCount == 1)                                                               // ...если мы в режиме порога срабатывания на подъёме
          {                                                                               //
            TresUPchange = true;                                                          //
            TrUPcount++;                                                                  //
            if (TrUPcount == 5) {TrUPcount = 0;}                                          //
          }                                                                               //
        if (menuCount == 2)                                                               // ...если мы в режиме порога срабатывания на снижении
          {                                                                               //
            TresDNchange = true;                                                          //
            TrDNcount++;                                                                  //
            if (TrDNcount == 5) {TrDNcount = 0;}                                          //
          }                                                                               //
        if (menuCount == 3)                                                               // ...если мы в режиме ВКЛ/ВЫКЛ светодиодов
          {                                                                               //
            if (LED_ON == true) {LED_ON = false;}                                         //
            else {LED_ON = true;}                                                         //
          }                                                                               //
        if (menuCount == 4)                                                               // ...если мы в режиме ВКЛ/ВЫКЛ звука клавиш
          {                                                                               //
            if (button_beep_ON == true) {button_beep_ON = false;}                         //
            else {button_beep_ON = true;}                                                 //
          }                                                                               //
        if (menuCount == 5)                                                               // ...если мы в режиме громкости пищалки
          {                                                                               //
            volCount++;                                                                   //
            if (volCount == 4) {volCount = 0;}                                            // после MAX снова OFF
            playTone(1800,60);                                                            // пробный сигнал на новой громкости
            beepDelay(60);                                                                //
          }                                                                               //
        if (menuCount == 6)                                                               // ...если мы в режиме выбора работы вариометра
          {                                                                               //
            if (Thermal_ON == true) {Thermal_ON = false;}                                 //
            else {Thermal_ON = true;}                                                     //
          }                                                                               //
        if (menuCount == 7)                                                               // ...если мы в режиме времени усреднения
          {                                                                               //
            AvgCount++;                                                                   //
            if (AvgCount == 5) {AvgCount = 0;}                                            // после последнего значения - снова первое
            avgFill = 0;                                                                  // начинаем накопление заново
          }                                                                               //
      }                                                                                   //
  }                                                                                       //

///// ДЛИННОЕ НАЖАТИЕ КНОПКИ 3 / BUTTON 3 LONG PRESS /////
if (Buttom3.click())                                                                      // если кнопка 3 зажата больше 1 секунды, то... (длительное нажатие)
  {                                                                                       //
    triple_beep();                                                                        // вызов функции ТРОЙНОГО ПИСКА
    if (MainMenu == false)                                                                // ... если мы не в МЕНЮ...
      {
        if (windowCount == 0)                                                             // если мы в первом главном окне
          {                                                                               // ОТКЛЮЧАЕМ/ВКЛЮЧАЕМ звук вариометра
            if (beep_ON == true) {beep_ON = false;}                                       // если звук включен, то выключаем
            else {beep_ON = true;}                                                        // иначе включаем
          }                                                                               //
        if (windowCount == 1)                                                             // если мы во втором главном окне
          {                                                                               // ОБНУЛЕНИЕ МАКСИМАЛЬНЫХ ЗНАЧЕНИЙ ВЫСОТ
            lcd.clear();                                                                  // чистим экран
            maxAltSea = 0;                                                                // сбрасываем на ноль максимальную высоту над морем
            maxAltStt = 0;                                                                // сбрасываем на ноль максимальную высоту над стартом
            maxAltAdd = 0;                                                                // сбрасываем на ноль максимальную дополнительную высоту
          }                                                                               //
        if (windowCount == 2)                                                             // если мы в третьем главном окне
          {                                                                               // ОБНУЛЕНИЕ МАКСИМАЛЬНЫХ ЗНАЧЕНИЙ ВАРИОМЕТРА
            lcd.clear();                                                                  // чистим экран
            maxVario = 0;                                                                 // сбрасываем на ноль максимальное значение вариометра
            minVario = 0;                                                                 // сбрасываем на ноль минимальное значение вариометра
          }                                                                               //
        if (windowCount == 3)                                                             // если мы на четвёртом окне...
          {                                                                               // ...обнуляем усреднённые макс/мин
            lcd.clear();                                                                  //
            maxVarioAvg = 0;                                                              //
            minVarioAvg = 0;                                                              //
          }                                                                               //
      }                                                                                   //
    else if (askSave == false)                                                                                  // ... если мы, все-таки, в МЕНЮ, то...
      {                                                                                   // при длительном нажатии КНОПКИ 3 в режиме МЕНЮ происходит сброс параметров к дефолтным
        presSea = 1013.25;                                                                 // переменной для формулы расчета высоты присваивается начальное значение
        presTMP = 1013.25;                                                                 // временной переменной давления присваивается начальное значение
        s0 = '1';                                                                         // посимвольное присвоение строкам начальных значений давления
        s1 = '0';                                                                         // посимвольное присвоение строкам начальных значений давления
        s2 = '1';                                                                         // посимвольное присвоение строкам начальных значений давления
        s3 = '3';                                                                         // посимвольное присвоение строкам начальных значений давления
        s4 = '2';                                                                         // посимвольное присвоение строкам начальных значений давления
        altSEATmp = 0;                                                                    // обнуление корректируемой высоты (нужно, что бы, при ранее обнулённой высоте у моря, значение вернулось к расчетному по давлению)
      }                                                                                   //
    longpress = true;                                                                     // переменной присваивается true, т.е. факт длительного нажатия
  }                                                                                       //
if (b3.fell() == 1) {longpress = false;}                                                  // убираем факт длительного нажатия при отпускании книпки
Buttom3.release();                                                                        // обязательно надо прописывать после выполнения click(), иначе выполнится только раз

///// ВЫЧИСЛЕНИЕ ВЫСОТЫ / ALTITUDE CALCULATING /////
altitude = sensor.alti-altTmp;                                                            // ВЫСОТА СТАРТА - присвоение переменной altitude (высота старта или дополнительная высота) значения высоты с датчика минус высота в точке обнуления
altitudeAdd = sensor.alti-altTmpAdd;                                                      // ДОПОЛНИТЕЛЬНАЯ ВЫСОТА - присвоение переменной altitudeAdd (высота старта или дополнительная высота) значения высоты с датчика минус высота в точке обнуления
altitudeSEA = -29.21*float(log((sensor.pres*MMHG_TO_HPA)/presSea))*(sensor.temp+273.15);                // ВЫСОТА ПО УРОВНЮ МОРЯ - вычисление по приведённому давлению, давлению в данный момент и значению температуры

///// УСРЕДНЕНИЕ ПОКАЗАНИЙ ВАРИОМЕТРА ДЛЯ ЭКРАНА / AVERAGED VARIO FOR THE SCREEN /////
if (millis() - avgTime >= 1000)                                         // раз в секунду запоминаем высоту
  {
    avgTime = millis();                                                 //
    avgIdx++;                                                           //
    if (avgIdx == 31) {avgIdx = 0;}                                     //
    avgAlt[avgIdx] = sensor.alti;                                       //
    if (avgFill < 31) {avgFill++;}                                      //
    byte n = avgSec[AvgCount];                                          // выбранное время усреднения
    if (avgFill > n)                                                    // когда снимков накопилось достаточно...
      {
        byte old = (avgIdx + 31 - n) % 31;                              // снимок n секунд назад
        varioAvg = (avgAlt[avgIdx] - avgAlt[old]) / float(n);           // средняя скорость за это время
      }
  }

///// ПРИСВОЕНИЕ ЗНАЧЕНИЙ МАКС/МИН ВАРИОМЕТРА / ASSIGNING VALUES FOR MAX/MIN VARIO /////
if (maxVario<vario) {maxVario = vario;}                                 // присвоение максимального значения вариометра
if (minVario>vario) {minVario = vario;}                                 // присвоение минимального значения вариометра
if (maxVarioAvg < varioAvg) {maxVarioAvg = varioAvg;}               // запоминаем максимум усреднённого значения
if (minVarioAvg > varioAvg) {minVarioAvg = varioAvg;}               // запоминаем минимум усреднённого значения

///// ПРИСВОЕНИЕ ЗНАЧЕНИЙ МАКСИМАЛЬНОЙ ВЫСОТЫ / ASSIGNING VALUES FOR MAX ALTITUDES /////
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

///// НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ ПЕРЕМЕННЫХ УСРЕДНИТЕЛЯ / INITIAL VALUES FOR AVERAGER /////
tempo = millis();                                                       // описание переменной tempo типа float и присвоение ей значения функции millis() - отсчитывает время от начала работы программы в миллисекундах 
N1 = 0;                                                                 // переменная для работы усреднителя
N2 = 0;                                                                 // переменная для работы усреднителя
N3 = 0;                                                                 // переменная для работы усреднителя
D1 = 0;                                                                 // переменная для работы усреднителя
D2 = 0;                                                                 // переменная для работы усреднителя

///// ОБНУЛЕНИЕ ВАРИОМЕТРА / ZERO VARIO /////
vario = 0;                                                              // обнуляем вариометр перед расчётом

///// ЦИКЛ УСРЕДНИТЕЛЯ / AVERAGER CYCLE /////
for(int cc=1; cc<=maxsamples; cc++)                                     // УСРЕДНИТЕЛЬ 
{                                                                       // За него огромное спасибо авторам этих проектов:
  alt[(cc-1)] = alt[cc];                                                // http://www.instructables.com/id/GoFly-paraglidinghanglidinggliding-altimeter-v/?ALLSTEPS
  tim[(cc-1)] = tim[cc];                                                // http://redhats.ru/variometer-arduino-2015/
}                                                                       //
                                                                        //
alt[maxsamples] = sensor.alti;                                          //
tim[maxsamples] = tempo;                                                //
float stime = tim[maxsamples-samples];                                  //
                                                                        //
for(int cc=(maxsamples-samples); cc<maxsamples; cc++)                   //
{                                                                       //
  N1+=(tim[cc]-stime)*alt[cc];                                          //
  N2+=(tim[cc]-stime);                                                  //
  N3+=(alt[cc]);                                                        //
  D1+=(tim[cc]-stime)*(tim[cc]-stime);                                  //
  D2+=(tim[cc]-stime);                                                  //
}                                                                       // окончание тела усреднителя 

///// РАСЧЁТ ЗНАЧЕНИЙ ВАРИОМЕТРА / VARIO VALUES CALCULATING /////
vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);                     // РАСЧЁТ ЗНАЧЕНИЙ ВАРИОМЕТРА 

///// ВЫВОД ЗВУКА / VARIO SOUND OUTPUT /////
if (beep_ON == true)                                                    // если включён режим СО ЗВУКОМ...
{                                                                       //
if ((tempo-beep)>Beep_period)                                           // 
{                                                                       //
  beep=tempo;                                                           //
  if (vario>vario_up && vario<15)                                       // звук на подъёме
  {                                                                     //
    Alarm = false;                                                      // отключаем тревогу
    Stmp = true;                                                        // переменная для отображения символа "B" во время работы пищалки
    gLEDtmp = true;                                                     // зеленый светодиод на подъеме
    Beep_period = 350-(vario*80);                                       //
    playTone((600+(250*vario)),300-(vario*80));                          // выводим звук (было 1800)
  }                                                                     //
  else if ((vario<vario_down))                      // звук на сливе
  {                                                                     //
    Alarm = false;                                                      // отключаем тревогу
    Stmp = true;                                                        // переменная для отображения символа "B" во время работы пищалки
    rLEDtmp = true;                                                     // красный светодиод на сливе
    Beep_period=40;                                                     // update pitch every 40 ms (continuous tone)
    playTone(constrain(int(600+(110*vario)),260,900),150);   // continuous sink tone: lower pitch the faster you sink
  }                                                                     //
}                                                                       //
}                                                                       //
else                                                                    // ... иначе просто выводим символ "B" рядом со значением вариометра и зажигаем светодиоды
{                                                                       //
if ((tempo-beep)>Beep_period)                                           // 
{                                                                       //
  beep=tempo;                                                           //
  if (vario>vario_up && vario<15 )                                      // 15 берём чисто из рандомных побуждений
  {                                                                     //
    Stmp = true;                                                        // "B" на подъёме
    gLEDtmp = true;                                                     // зеленый светодиод на подъеме
  }                                                                     //
  else if ((vario<vario_down))                      // 
  {                                                                     //
    Stmp = true;                                                        // "B" на сливе
    rLEDtmp = true;                                                     // красный светодиод на сливе
  }                                                                     //
}                                                                       //
}                                                                       //

///// ЗАЖИГАЕМ/ГАСИМ СВЕТОДИОДЫ / LED ON/OFF FOR VARIO /////
if ((LED_ON == true)&&(gLEDtmp == true))                                // если (режим с подсветкой включен) и (разрешен зеленый светодиод)
  {digitalWrite(gLED, HIGH); delay(10); digitalWrite(gLED, LOW);}       // зажигаем зеленый светодиод, ждём время delay, гасим светодиод (гасим быстро, что б батарейку не расходовать)
else if ((LED_ON == false)||(gLEDtmp == false))                         // иначе, если (режим с подсветкой вЫключен) ИЛИ (запрещен зеленый светодиод)
  {digitalWrite(gLED, LOW);}                                            // гасим зеленый светодиод
if ((LED_ON == true)&&(rLEDtmp == true))                                // если (режим с подсветкой включен) и (разрешен красный светодиод)
  {digitalWrite(rLED, HIGH); delay(10); digitalWrite(gLED, LOW);}       // зажигаем красный светодиод, ждём время delay, гасим светодиод (гасим быстро, что б батарейку не расходовать)
else if ((LED_ON == true)||(rLEDtmp == false))                          // иначе, если (режим с подсветкой вЫключен) ИЛИ (запрещен красный светодиод)
  {digitalWrite(rLED, LOW);}                                            // гасим красный светодиод


///// ЗАПУСК ПОДПРОГРАММ, В ЗАВИСИМОСТИ ОТ ВКЛ/ВЫКЛ МЕНЮ / START MAIN WINDOW OR MENU /////
if (MainMenu == false)                                                  // Если режим МЕНЮ ВЫКЛЮЧЕН, то...
  {lcd_out();}                                                          // подпрограмма вывода значений на экран (НЕ МЕНЮ)
else                                                                    // ... иначе ...
  {Menu();}                                                             // очищаем экран и запускаем подпрограмму МЕНЮ

///// ОБНУЛЕНИЕ ПЕРЕМЕННЫХ В КОНЦЕ ЦИКЛА / VALUES TO ZERO IN THE END OF THE CYCLE /////
Stmp = false;                                                           // переменная для отображения символа "B" во время работы пищалки
gLEDtmp = false;                                                        // выключаем зеленый светодиод
rLEDtmp = false;                                                        // выключаем красный светодиод
}

///// ВЫВОД СООБЩЕНИЯ ОБ ОШИБКЕ / ERROR MESSAGE IF SOMETHING WRONG WITH BMP180 SENSOR /////
else                                                                    //
  {                                                                     //
    lcd.setCursor(0,0);                                                 //
    lcd.print("Sensor error");                                          // вывод на экран информации об ошибке, если данные с датчика не поступают 
  }                                                                     //

//Serial.println((String)" samples="+samples+" maxsamples="+maxsamples);
}

