#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "Adafruit_PM25AQI.h"

// ---- Pin mapping ----
#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define JOY_BTN_PIN 7
#define MQ135_PIN A2
#define FLAME_PIN A3
#define DHTPIN 4
#define DHTTYPE DHT22
#define BUZZER_PIN 5
#define RED 6
#define GREEN 10
#define BLUE 11
#define PM25_RX_PIN 2
#define PM25_TX_PIN 3
#define FAN_PIN 9

// PM serial compatibility:
// - Prefer hardware Serial1 (Uno R4 / boards that expose it)
// - Fallback to SoftwareSerial when available
// - If neither exists, compile with PM disabled
#if defined(ARDUINO_ARCH_RENESAS) || defined(HAVE_HWSERIAL1)
#define PM_HAS_SERIAL 1
#define PM_USE_HARDWARE_SERIAL 1
#elif __has_include(<SoftwareSerial.h>)
#include <SoftwareSerial.h>
#define PM_HAS_SERIAL 1
#define PM_USE_HARDWARE_SERIAL 0
#else
#define PM_HAS_SERIAL 0
#define PM_USE_HARDWARE_SERIAL 0
#endif

// ---- Timing ----
const unsigned long FAST_SENSOR_MS = 120;
const unsigned long SLOW_SENSOR_MS = 1200;
const unsigned long STATE_MS = 160;
const unsigned long OUTPUT_MS = 80;
const unsigned long UI_MS = 220;
const unsigned long JOY_REPEAT_MS = 220;
const unsigned long MENU_TIMEOUT_MS = 15000;

const uint8_t FLAME_FILTER_N = 8;
const int JOY_LOW = 260;
const int JOY_HIGH = 760;

LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPIN, DHTTYPE);
Adafruit_PM25AQI aqi;
#if PM_HAS_SERIAL && PM_USE_HARDWARE_SERIAL
HardwareSerial& pmSerial = Serial1;
#elif PM_HAS_SERIAL
SoftwareSerial pmSerial(PM25_RX_PIN, PM25_TX_PIN);
#endif

int fanDuty = 10;

enum AirLevel { AIR_GREEN = 0, AIR_YELLOW = 1, AIR_ORANGE = 2, AIR_RED = 3 };
enum SystemState { IDLE_CLOCK = 0, MENU = 1, ALERT_ORANGE = 2, ALERT_RED = 3, FIRE_ALERT = 4 };
enum UiPage { PAGE_GAS = 0, PAGE_PM = 1, PAGE_TEMP = 2, PAGE_FLAME = 3, PAGE_FAN = 4, PAGE_CLOCK = 5, PAGE_COUNT = 6 };
enum OutlierSource { SRC_GAS = 0, SRC_PM25 = 1, SRC_PM10 = 2 };

struct SensorData {
  int mqRaw;
  uint16_t gasPpmEst;
  int flameRaw;
  int flameAvg;
  bool flameDetected;
  float tempC;
  float humidity;
  bool dhtValid;
  PM25_AQI_Data pm;
  bool pmValid;
};

struct ThresholdConfig {
  int mqYellow;
  int mqOrange;
  int mqRed;
  uint16_t pm25Yellow;
  uint16_t pm25Orange;
  uint16_t pm25Red;
  uint16_t pm10Yellow;
  uint16_t pm10Orange;
  uint16_t pm10Red;
  int flameTrigger;
  int flameHysteresis;
  uint8_t fanDutyByLevel[4]; // green, yellow, orange, red
};

struct ClockData {
  uint8_t hh;
  uint8_t mm;
  uint8_t ss;
  bool editMode;
  uint8_t field; // 0:hh, 1:mm, 2:ss
};

SensorData sensor = {0, 10, 0, 0, false, NAN, NAN, false, {}, false};
ThresholdConfig cfg = {
  350, 550, 750, // MQ thresholds
  12, 35, 55,    // PM2.5 thresholds
  54, 154, 254,  // PM10 thresholds
  380, 40,       // Flame trigger + hysteresis
  {20, 35, 65, 90}
};
ClockData clockData = {12, 0, 0, false, 0};

bool pmReady = false;
SystemState systemState = IDLE_CLOCK;
AirLevel airLevel = AIR_GREEN;
UiPage uiPage = PAGE_GAS;
OutlierSource outlierSrc = SRC_GAS;
int outlierValue = 0;
bool menuActive = false;

int joyX = 512;
int joyY = 512;
bool joyPressed = false;
bool prevJoyPressed = false;

int flameBuf[FLAME_FILTER_N];
uint8_t flameIdx = 0;
long flameSum = 0;
bool flamePrimed = false;

unsigned long lastFastRead = 0;
unsigned long lastSlowRead = 0;
unsigned long lastStateTick = 0;
unsigned long lastOutputTick = 0;
unsigned long lastUiTick = 0;
unsigned long lastInputTick = 0;
unsigned long lastUserInput = 0;
unsigned long lastClockTick = 0;
unsigned long blinkMark = 0;
bool blinkOn = true;
unsigned long lastTraceTickClock = 0;
unsigned long lastTraceRead = 0;
unsigned long lastTraceInput = 0;
unsigned long lastTraceState = 0;
unsigned long lastTraceOutput = 0;
unsigned long lastTraceUi = 0;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED, r);
  analogWrite(GREEN, g);
  analogWrite(BLUE, b);
}

// Uno Q/R4 is non-AVR, so avoid AVR Timer1 registers.
// Use portable PWM setup that works across Uno families.
void setupFanPwm() {
  pinMode(FAN_PIN, OUTPUT);
}

void setFanDuty(int duty) {
  if (duty < 0) duty = 0;
  if (duty > 100) duty = 100;
  fanDuty = duty;
  uint8_t pwmValue = (uint8_t)map(fanDuty, 0, 100, 0, 255);
  analogWrite(FAN_PIN, pwmValue);
}

uint16_t estimatePpmFromMq(int raw) {
  long ppm = map(raw, 80, 900, 10, 1000);
  if (ppm < 10) ppm = 10;
  if (ppm > 1000) ppm = 1000;
  return (uint16_t)ppm;
}

AirLevel levelFromThresholds(int value, int yellow, int orange, int red) {
  if (value >= red) return AIR_RED;
  if (value >= orange) return AIR_ORANGE;
  if (value >= yellow) return AIR_YELLOW;
  return AIR_GREEN;
}

const char* levelName(AirLevel l) {
  if (l == AIR_GREEN) return "GREEN";
  if (l == AIR_YELLOW) return "YELLOW";
  if (l == AIR_ORANGE) return "ORANGE";
  return "RED";
}

void updateFlameFilter(int raw) {
  if (!flamePrimed) {
    for (uint8_t i = 0; i < FLAME_FILTER_N; i++) flameBuf[i] = raw;
    flameSum = (long)raw * FLAME_FILTER_N;
    flamePrimed = true;
    sensor.flameAvg = raw;
    return;
  }
  flameSum -= flameBuf[flameIdx];
  flameBuf[flameIdx] = raw;
  flameSum += raw;
  flameIdx = (flameIdx + 1) % FLAME_FILTER_N;
  sensor.flameAvg = (int)(flameSum / FLAME_FILTER_N);
}

void updateFlameDetection() {
  // KY-026 analog typically decreases when flame IR increases.
  if (!sensor.flameDetected && sensor.flameAvg <= cfg.flameTrigger) {
    sensor.flameDetected = true;
  } else if (sensor.flameDetected &&
             sensor.flameAvg >= (cfg.flameTrigger + cfg.flameHysteresis)) {
    sensor.flameDetected = false;
  }
}

void tickClock(unsigned long now) {
  if (now - lastTraceTickClock >= 2000UL) {
    lastTraceTickClock = now;
    Serial.println(F("[TASK] tickClock"));
  }

  while (now - lastClockTick >= 1000UL) {
    lastClockTick += 1000UL;
    clockData.ss++;
    if (clockData.ss >= 60) {
      clockData.ss = 0;
      clockData.mm++;
      if (clockData.mm >= 60) {
        clockData.mm = 0;
        clockData.hh = (clockData.hh + 1) % 24;
      }
    }
  }
}

void adjustClockField(int delta) {
  if (clockData.field == 0) {
    int hh = (int)clockData.hh + delta;
    if (hh < 0) hh = 23;
    if (hh > 23) hh = 0;
    clockData.hh = (uint8_t)hh;
  } else if (clockData.field == 1) {
    int mm = (int)clockData.mm + delta;
    if (mm < 0) mm = 59;
    if (mm > 59) mm = 0;
    clockData.mm = (uint8_t)mm;
  } else {
    int ss = (int)clockData.ss + delta;
    if (ss < 0) ss = 59;
    if (ss > 59) ss = 0;
    clockData.ss = (uint8_t)ss;
  }
}

void readSensorsTask(unsigned long now) {
  if (now - lastTraceRead >= 2000UL) {
    lastTraceRead = now;
    Serial.println(F("[TASK] readSensorsTask"));
  }

  if (now - lastFastRead >= FAST_SENSOR_MS) {
    lastFastRead = now;
    sensor.mqRaw = analogRead(MQ135_PIN);
    sensor.gasPpmEst = estimatePpmFromMq(sensor.mqRaw);
    sensor.flameRaw = analogRead(FLAME_PIN);
    updateFlameFilter(sensor.flameRaw);
    updateFlameDetection();

    joyX = analogRead(JOY_X_PIN);
    joyY = analogRead(JOY_Y_PIN);
    joyPressed = (digitalRead(JOY_BTN_PIN) == LOW);
  }

  if (now - lastSlowRead >= SLOW_SENSOR_MS) {
    lastSlowRead = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      sensor.dhtValid = false;
    } else {
      sensor.humidity = h;
      sensor.tempC = t;
      sensor.dhtValid = true;
    }

    if (PM_HAS_SERIAL && pmReady && pmSerial.available() >= 32) {
      sensor.pmValid = aqi.read(&sensor.pm);
    } else {
      sensor.pmValid = false;
    }
  }
}

void handleInputTask(unsigned long now) {
  if (now - lastTraceInput >= 2000UL) {
    lastTraceInput = now;
    Serial.println(F("[TASK] handleInputTask"));
  }

  bool btnEvent = joyPressed && !prevJoyPressed;
  prevJoyPressed = joyPressed;

  if (btnEvent) {
    lastInputTick = now;
    lastUserInput = now;
    if (!menuActive) {
      menuActive = true;
      uiPage = PAGE_GAS;
      Serial.println(F("[INPUT] menu open"));
      return;
    }

    if (uiPage == PAGE_CLOCK) {
      if (!clockData.editMode) {
        clockData.editMode = true;
        clockData.field = 0;
        Serial.println(F("[INPUT] clock edit start"));
      } else if (clockData.field < 2) {
        clockData.field++;
        Serial.println(F("[INPUT] clock next field"));
      } else {
        clockData.editMode = false;
        Serial.println(F("[INPUT] clock saved"));
      }
    } else {
      menuActive = false;
      Serial.println(F("[INPUT] menu exit"));
    }
  }

  if (!menuActive) return;
  if (now - lastInputTick < JOY_REPEAT_MS) return;

  if (!clockData.editMode && joyY > JOY_HIGH) {
    lastInputTick = now;
    lastUserInput = now;
    if (uiPage == PAGE_GAS) uiPage = (UiPage)(PAGE_COUNT - 1);
    else uiPage = (UiPage)(uiPage - 1);
    Serial.println(F("[INPUT] page up"));
    return;
  }
  if (!clockData.editMode && joyY < JOY_LOW) {
    lastInputTick = now;
    lastUserInput = now;
    uiPage = (UiPage)((uiPage + 1) % PAGE_COUNT);
    Serial.println(F("[INPUT] page down"));
    return;
  }

  if (joyX > JOY_HIGH || joyX < JOY_LOW) {
    lastInputTick = now;
    lastUserInput = now;
    int delta = (joyX > JOY_HIGH) ? 1 : -1;
    if (uiPage == PAGE_CLOCK && clockData.editMode) {
      adjustClockField(delta);
      lastClockTick = now;
      Serial.println(F("[INPUT] clock value change"));
    }
  }

  if (now - lastUserInput >= MENU_TIMEOUT_MS) {
    menuActive = false;
    clockData.editMode = false;
    Serial.println(F("[INPUT] menu timeout"));
  }
}

void updateStateTask(unsigned long now) {
  if (now - lastTraceState >= 2000UL) {
    lastTraceState = now;
    Serial.println(F("[TASK] updateStateTask"));
  }

  if (now - lastStateTick < STATE_MS) return;
  lastStateTick = now;

  AirLevel mqLevel = levelFromThresholds(sensor.mqRaw, cfg.mqYellow, cfg.mqOrange, cfg.mqRed);
  AirLevel pm25Level = AIR_GREEN;
  AirLevel pm10Level = AIR_GREEN;

  if (sensor.pmValid) {
    pm25Level = levelFromThresholds((int)sensor.pm.pm25_standard, cfg.pm25Yellow, cfg.pm25Orange, cfg.pm25Red);
    pm10Level = levelFromThresholds((int)sensor.pm.pm100_standard, cfg.pm10Yellow, cfg.pm10Orange, cfg.pm10Red);
  }

  airLevel = mqLevel;
  outlierSrc = SRC_GAS;
  outlierValue = sensor.mqRaw;
  if ((int)pm25Level > (int)airLevel) {
    airLevel = pm25Level;
    outlierSrc = SRC_PM25;
    outlierValue = sensor.pm.pm25_standard;
  }
  if ((int)pm10Level > (int)airLevel) {
    airLevel = pm10Level;
    outlierSrc = SRC_PM10;
    outlierValue = sensor.pm.pm100_standard;
  }

  if (sensor.flameDetected) {
    systemState = FIRE_ALERT;
  } else if (menuActive) {
    systemState = MENU;
  } else if (airLevel >= AIR_RED) {
    systemState = ALERT_RED;
  } else if (airLevel >= AIR_ORANGE) {
    systemState = ALERT_ORANGE;
  } else {
    systemState = IDLE_CLOCK;
  }
}

void applyLedPattern(unsigned long now) {
  uint16_t period = 0;
  if (systemState == ALERT_ORANGE) period = 500;
  if (systemState == ALERT_RED) period = 250;
  if (systemState == FIRE_ALERT) period = 120;

  if (period > 0 && now - blinkMark >= period) {
    blinkMark = now;
    blinkOn = !blinkOn;
  } else if (period == 0) {
    blinkOn = true;
  }

  if (!blinkOn && period > 0) {
    setColor(0, 0, 0);
    return;
  }

  if (systemState == FIRE_ALERT) {
    setColor(255, 0, 0);
    return;
  }
  if (airLevel == AIR_GREEN) setColor(0, 255, 0);
  if (airLevel == AIR_YELLOW) setColor(255, 180, 0);
  if (airLevel == AIR_ORANGE) setColor(255, 80, 0);
  if (airLevel == AIR_RED) setColor(255, 0, 0);
}

void applyBuzzerPattern(unsigned long now) {
  static unsigned long lastTone = 0;
  static bool fireFlip = false;

  if (systemState == FIRE_ALERT) {
    if (now - lastTone >= 180) {
      lastTone = now;
      fireFlip = !fireFlip;
      tone(BUZZER_PIN, fireFlip ? 2000 : 1400, 140);
    }
    return;
  }
  if (systemState == ALERT_RED) {
    if (now - lastTone >= 500) {
      lastTone = now;
      tone(BUZZER_PIN, 1300, 220);
    }
    return;
  }
  if (systemState == ALERT_ORANGE) {
    if (now - lastTone >= 900) {
      lastTone = now;
      tone(BUZZER_PIN, 900, 150);
    }
    return;
  }
  noTone(BUZZER_PIN);
}

void applyFanControl() {
  if (systemState == FIRE_ALERT) {
    setFanDuty(100);
    return;
  }
  setFanDuty(cfg.fanDutyByLevel[(int)airLevel]);
}

void updateOutputTask(unsigned long now) {
  if (now - lastTraceOutput >= 2000UL) {
    lastTraceOutput = now;
    Serial.println(F("[TASK] updateOutputTask"));
  }

  if (now - lastOutputTick < OUTPUT_MS) return;
  lastOutputTick = now;
  applyLedPattern(now);
  applyBuzzerPattern(now);
  applyFanControl();
}

void renderIdleClock() {
  lcd.setCursor(0, 0);
  lcd.print("Clock ");
  if (clockData.hh < 10) lcd.print('0');
  lcd.print(clockData.hh);
  lcd.print(':');
  if (clockData.mm < 10) lcd.print('0');
  lcd.print(clockData.mm);
  lcd.print(':');
  if (clockData.ss < 10) lcd.print('0');
  lcd.print(clockData.ss);
  lcd.print("   ");

  lcd.setCursor(0, 1);
  lcd.print("Air:");
  lcd.print(levelName(airLevel));
  lcd.print(" Fan:");
  lcd.print(fanDuty);
  lcd.print("%   ");

  lcd.setCursor(0, 2);
  if (outlierSrc == SRC_GAS) lcd.print("Outlier:Gas ");
  if (outlierSrc == SRC_PM25) lcd.print("Outlier:PM2.5");
  if (outlierSrc == SRC_PM10) lcd.print("Outlier:PM10 ");
  lcd.print(" ");
  lcd.print(outlierValue);
  lcd.print("   ");

  lcd.setCursor(0, 3);
  lcd.print("Btn=Menu AutoAlertOn ");
}

void renderGasPage() {
  lcd.setCursor(0, 0); lcd.print("Gas Sensor MQ-135    ");
  lcd.setCursor(0, 1); lcd.print("Raw ADC: "); lcd.print(sensor.mqRaw); lcd.print("       ");
  lcd.setCursor(0, 2); lcd.print("Est ppm: "); lcd.print(sensor.gasPpmEst); lcd.print(" 10-1000 ");
  lcd.setCursor(0, 3); lcd.print("Lvl:"); lcd.print(levelName(airLevel)); lcd.print(" Btn=Exit ");
}

void renderPmPage() {
  lcd.setCursor(0, 0); lcd.print("PM Sensor PMS5003    ");
  if (!sensor.pmValid) {
    lcd.setCursor(0, 1); lcd.print("PM read unavailable  ");
    lcd.setCursor(0, 2); lcd.print("Check D2/D3 wiring   ");
    lcd.setCursor(0, 3); lcd.print("Y nav Btn=Exit       ");
    return;
  }

  lcd.setCursor(0, 1);
  lcd.print("M1:");
  lcd.print(sensor.pm.pm10_standard);
  lcd.print(" M2:");
  lcd.print(sensor.pm.pm25_standard);
  lcd.print(" M10:");
  lcd.print(sensor.pm.pm100_standard);
  lcd.print(" ");

  lcd.setCursor(0, 2);
  lcd.print("C03:");
  lcd.print(sensor.pm.particles_03um);
  lcd.print(" C05:");
  lcd.print(sensor.pm.particles_05um);
  lcd.print(" ");

  lcd.setCursor(0, 3);
  if ((clockData.ss & 1) == 0) {
    lcd.print("C1:");
    lcd.print(sensor.pm.particles_10um);
    lcd.print(" C2.5:");
    lcd.print(sensor.pm.particles_25um);
    lcd.print("  ");
  } else {
    lcd.print("C5:");
    lcd.print(sensor.pm.particles_50um);
    lcd.print(" C10:");
    lcd.print(sensor.pm.particles_100um);
    lcd.print("  ");
  }
}

void renderTempPage() {
  lcd.setCursor(0, 0); lcd.print("Temperature/Humidity ");
  if (!sensor.dhtValid) {
    lcd.setCursor(0, 1); lcd.print("DHT22 read error     ");
    lcd.setCursor(0, 2); lcd.print("Check D4/5V/GND      ");
    lcd.setCursor(0, 3); lcd.print("Y nav Btn=Exit       ");
    return;
  }
  float tempF = sensor.tempC * 9.0 / 5.0 + 32.0;
  lcd.setCursor(0, 1); lcd.print("T:"); lcd.print(sensor.tempC, 1); lcd.print("C "); lcd.print(tempF, 1); lcd.print("F     ");
  lcd.setCursor(0, 2); lcd.print("RH:"); lcd.print(sensor.humidity, 1); lcd.print("%           ");
  lcd.setCursor(0, 3); lcd.print("Y nav Btn=Exit       ");
}

void renderFlamePage() {
  lcd.setCursor(0, 0); lcd.print("Flame Sensor KY-026  ");
  lcd.setCursor(0, 1); lcd.print("Raw:"); lcd.print(sensor.flameRaw); lcd.print(" Avg:"); lcd.print(sensor.flameAvg); lcd.print("   ");
  lcd.setCursor(0, 2); lcd.print("Trig:"); lcd.print(cfg.flameTrigger); lcd.print(" Hys:"); lcd.print(cfg.flameHysteresis); lcd.print("  ");
  lcd.setCursor(0, 3); lcd.print(sensor.flameDetected ? "FIRE " : "SAFE "); lcd.print("Read-only Btn=Exit ");
}

void renderFanPage() {
  lcd.setCursor(0, 0); lcd.print("Fan Config by Level  ");
  lcd.setCursor(0, 1);
  lcd.print("G:");
  lcd.print((int)cfg.fanDutyByLevel[0]);
  lcd.print("% Y:");
  lcd.print((int)cfg.fanDutyByLevel[1]);
  lcd.print("% O:");
  lcd.print((int)cfg.fanDutyByLevel[2]);
  lcd.print("%");
  lcd.setCursor(0, 2); lcd.print("R:"); lcd.print((int)cfg.fanDutyByLevel[3]); lcd.print("% Run:"); lcd.print(fanDuty); lcd.print("%  ");
  lcd.setCursor(0, 3); lcd.print("Auto control (locked)");
}

void renderClockPage() {
  lcd.setCursor(0, 0); lcd.print("Clock Setup          ");
  lcd.setCursor(0, 1);
  lcd.print("Time ");
  if (clockData.hh < 10) lcd.print('0');
  lcd.print(clockData.hh);
  lcd.print(':');
  if (clockData.mm < 10) lcd.print('0');
  lcd.print(clockData.mm);
  lcd.print(':');
  if (clockData.ss < 10) lcd.print('0');
  lcd.print(clockData.ss);
  lcd.print("     ");

  lcd.setCursor(0, 2);
  if (!clockData.editMode) {
    lcd.print("Btn=Edit  Y nav      ");
  } else {
    lcd.print("L/R adjust ");
    if (clockData.field == 0) lcd.print("HH ");
    if (clockData.field == 1) lcd.print("MM ");
    if (clockData.field == 2) lcd.print("SS ");
    lcd.print("     ");
  }
  lcd.setCursor(0, 3); lcd.print("Btn step/save+exit   ");
}

void renderAlertScreen() {
  lcd.setCursor(0, 0); lcd.print("Air Warning Active   ");
  lcd.setCursor(0, 1); lcd.print("Level: "); lcd.print(levelName(airLevel)); lcd.print("          ");
  lcd.setCursor(0, 2); lcd.print("Outlier: ");
  if (outlierSrc == SRC_GAS) lcd.print("Gas ");
  if (outlierSrc == SRC_PM25) lcd.print("PM2.5 ");
  if (outlierSrc == SRC_PM10) lcd.print("PM10 ");
  lcd.print(outlierValue);
  lcd.print("   ");
  lcd.setCursor(0, 3); lcd.print("Auto fan+alarm BtnUI ");
}

void renderFireScreen() {
  lcd.setCursor(0, 0); lcd.print("!!! FIRE ALERT !!!   ");
  lcd.setCursor(0, 1); lcd.print("Flame avg: "); lcd.print(sensor.flameAvg); lcd.print("       ");
  lcd.setCursor(0, 2); lcd.print("Fan forced to 100%   ");
  lcd.setCursor(0, 3); lcd.print("Btn=Menu             ");
}

void updateUiTask(unsigned long now) {
  if (now - lastTraceUi >= 2000UL) {
    lastTraceUi = now;
    Serial.println(F("[TASK] updateUiTask"));
  }

  if (now - lastUiTick < UI_MS) return;
  lastUiTick = now;

  if (!menuActive && systemState == FIRE_ALERT) {
    renderFireScreen();
    return;
  }
  if (!menuActive && (systemState == ALERT_ORANGE || systemState == ALERT_RED)) {
    renderAlertScreen();
    return;
  }
  if (!menuActive) {
    renderIdleClock();
    return;
  }

  if (uiPage == PAGE_GAS) renderGasPage();
  if (uiPage == PAGE_PM) renderPmPage();
  if (uiPage == PAGE_TEMP) renderTempPage();
  if (uiPage == PAGE_FLAME) renderFlamePage();
  if (uiPage == PAGE_FAN) renderFanPage();
  if (uiPage == PAGE_CLOCK) renderClockPage();
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("[BOOT] setup"));
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);

  setColor(0, 0, 0);
  noTone(BUZZER_PIN);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("PHYS124 Air Monitor  ");
  lcd.setCursor(0, 1); lcd.print("Sensor startup...    ");

  dht.begin();
  if (PM_HAS_SERIAL) {
    pmSerial.begin(9600);
    delay(1500);
    pmReady = aqi.begin_UART(&pmSerial);
  } else {
    pmReady = false;
  }

  setupFanPwm();
  setFanDuty(cfg.fanDutyByLevel[0]);

  sensor.flameRaw = analogRead(FLAME_PIN);
  updateFlameFilter(sensor.flameRaw);

  lastClockTick = millis();
  lastUserInput = lastClockTick;
  lcd.setCursor(0, 2); lcd.print(pmReady ? "PMS5003 online       " : "PMS5003 unavailable  ");
  lcd.setCursor(0, 3); lcd.print("Press joy btn for UI ");
  delay(1200);
  lcd.clear();
}

void loop() {
  unsigned long now = millis();
  tickClock(now);
  readSensorsTask(now);
  handleInputTask(now);
  updateStateTask(now);
  updateOutputTask(now);
  updateUiTask(now);
}
