#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include "Adafruit_PM25AQI.h"

// ---- Pin mappings (preserved from your working demo) ----
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

#define PM25_RX_PIN 2   // sensor TX -> Arduino pin 2
#define PM25_TX_PIN 3   // sensor RX -> Arduino pin 3 (unused in read-only mode)

#define FAN_PIN 9

// ---- Timing ----
const unsigned long SENSOR_FAST_MS = 120;
const unsigned long SENSOR_SLOW_MS = 1200;
const unsigned long STATE_MS = 150;
const unsigned long UI_MS = 250;
const unsigned long OUTPUT_MS = 80;
const unsigned long SERIAL_MS = 1000;
const unsigned long JOYSTICK_REPEAT_MS = 220;
const unsigned long MENU_IDLE_TIMEOUT_MS = 15000;
const unsigned long HEARTBEAT_MS = 2000;

// ---- Flame filter ----
const uint8_t FLAME_FILTER_N = 8;

// ---- Module objects ----
LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial pmSerial(PM25_RX_PIN, PM25_TX_PIN);
Adafruit_PM25AQI aqi;

// ---- Fan PWM setup (kept from existing sketch) ----
int icr = 639;   // 25 kHz on Timer1
int fanDuty = 10;

enum AirLevel {
  AIR_GREEN = 0,
  AIR_YELLOW = 1,
  AIR_ORANGE = 2,
  AIR_RED = 3
};

enum SystemState {
  IDLE_CLOCK = 0,
  MENU = 1,
  ALERT_ORANGE = 2,
  ALERT_RED = 3,
  FIRE_ALERT = 4
};

enum UiPage {
  PAGE_GAS = 0,
  PAGE_PM = 1,
  PAGE_TEMP_HUMID = 2,
  PAGE_FLAME = 3,
  PAGE_FAN = 4,
  PAGE_COUNT = 5
};

struct SensorData {
  int mqRaw;
  int flameRaw;
  int flameAvg;
  bool flameDetected;
  float tempC;
  float humidity;
  bool dhtValid;
  PM25_AQI_Data pm;
  bool pmValid;
  uint16_t gasPpmEst;
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

SensorData sensor = {0, 0, 0, false, NAN, NAN, false, {}, false, 0};
ThresholdConfig cfg = {
  350, 550, 750,
  12, 35, 55,
  54, 154, 254,
  380, 40,
  {20, 35, 65, 90}
};

// ---- Runtime state ----
bool pmReady = false;
SystemState systemState = IDLE_CLOCK;
AirLevel airLevel = AIR_GREEN;
UiPage uiPage = PAGE_GAS;
uint8_t fanConfigIndex = 0; // which level we edit on fan page
bool menuActive = false;

int joyX = 512;
int joyY = 512;
bool joyPressed = false;
bool prevJoyPressed = false;
unsigned long lastJoyAction = 0;
unsigned long lastUserInput = 0;

int flameBuf[FLAME_FILTER_N];
uint8_t flameIdx = 0;
long flameSum = 0;
bool flameBufPrimed = false;

unsigned long lastFastRead = 0;
unsigned long lastSlowRead = 0;
unsigned long lastStateUpdate = 0;
unsigned long lastUiUpdate = 0;
unsigned long lastOutputUpdate = 0;
unsigned long lastSerialUpdate = 0;
unsigned long lastHeartbeat = 0;

unsigned long blinkMark = 0;
bool blinkOn = false;

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED, r);
  analogWrite(GREEN, g);
  analogWrite(BLUE, b);
}

void setupFan25kHz() {
  pinMode(FAN_PIN, OUTPUT);
  TCCR1A = 0xA2;
  TCCR1B = 0x19;
  TCNT1H = 0x00;
  TCNT1L = 0x00;
  ICR1H = icr >> 8;
  ICR1L = icr & 0x00ff;
}

void setFanDuty(int duty) {
  if (duty < 0) duty = 0;
  if (duty > 100) duty = 100;
  fanDuty = duty;
  OCR1A = (uint16_t)(icr * (fanDuty / 100.0));
}

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

uint16_t estimatePpmFromMq(int mqRaw) {
  // Rough linear estimate for display only.
  long scaled = map(mqRaw, 80, 900, 10, 1000);
  if (scaled < 10) scaled = 10;
  if (scaled > 1000) scaled = 1000;
  return (uint16_t)scaled;
}

AirLevel levelFromThresholds(int value, int yellow, int orange, int red) {
  if (value >= red) return AIR_RED;
  if (value >= orange) return AIR_ORANGE;
  if (value >= yellow) return AIR_YELLOW;
  return AIR_GREEN;
}

void updateFlameFilter(int raw) {
  if (!flameBufPrimed) {
    for (uint8_t i = 0; i < FLAME_FILTER_N; i++) {
      flameBuf[i] = raw;
    }
    flameSum = (long)raw * FLAME_FILTER_N;
    flameBufPrimed = true;
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
  // KY-026 analog output typically drops with stronger flame IR.
  if (!sensor.flameDetected && sensor.flameAvg <= cfg.flameTrigger) {
    sensor.flameDetected = true;
  } else if (sensor.flameDetected &&
             sensor.flameAvg >= (cfg.flameTrigger + cfg.flameHysteresis)) {
    sensor.flameDetected = false;
  }
}

const char* levelName(AirLevel level) {
  if (level == AIR_GREEN) return "GREEN";
  if (level == AIR_YELLOW) return "YELLOW";
  if (level == AIR_ORANGE) return "ORANGE";
  return "RED";
}

void readSensorsTask(unsigned long now) {
  if (now - lastFastRead >= SENSOR_FAST_MS) {
    lastFastRead = now;
    sensor.mqRaw = analogRead(MQ135_PIN);
    sensor.flameRaw = analogRead(FLAME_PIN);
    updateFlameFilter(sensor.flameRaw);
    updateFlameDetection();

    joyX = analogRead(JOY_X_PIN);
    joyY = analogRead(JOY_Y_PIN);
    joyPressed = (digitalRead(JOY_BTN_PIN) == LOW);
  }

  if (now - lastSlowRead >= SENSOR_SLOW_MS) {
    lastSlowRead = now;
    Serial.println("Slow sensor cycle start");

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      sensor.dhtValid = false;
      Serial.println("DHT22 read failed");
    } else {
      sensor.humidity = h;
      sensor.tempC = t;
      sensor.dhtValid = true;
      Serial.println("DHT22 read success");
    }

    if (pmReady) {
      int availableBytes = pmSerial.available();
      Serial.print("PM serial bytes available: ");
      Serial.println(availableBytes);
      if (availableBytes >= 32) {
        Serial.println("PM read start");
        sensor.pmValid = aqi.read(&sensor.pm);
        Serial.println(sensor.pmValid ? "PM read success" : "PM read failed");
      } else {
        sensor.pmValid = false;
        Serial.println("PM read skipped (insufficient bytes)");
      }
    } else {
      sensor.pmValid = false;
      Serial.println("PM not ready");
    }

    sensor.gasPpmEst = estimatePpmFromMq(sensor.mqRaw);
    Serial.print("MQ raw: ");
    Serial.println(sensor.mqRaw);
    Serial.print("Flame raw: ");
    Serial.println(sensor.flameRaw);
    Serial.println("Slow sensor cycle end");
  }
}

void handleInputTask(unsigned long now) {
  bool btnEvent = (joyPressed && !prevJoyPressed);
  prevJoyPressed = joyPressed;

  if (btnEvent && (now - lastJoyAction >= 120)) {
    lastJoyAction = now;
    lastUserInput = now;
    if (!menuActive) {
      menuActive = true;
      systemState = MENU;
    } else {
      if (uiPage == PAGE_FAN) {
        if (fanConfigIndex < 3) {
          fanConfigIndex++;
        } else {
          fanConfigIndex = 0;
          menuActive = false;
        }
      } else {
        menuActive = false;
      }
    }
  }

  bool moved = false;
  if (now - lastJoyAction >= JOYSTICK_REPEAT_MS) {
    if (joyY > 760) {
      moved = true;
      lastJoyAction = now;
      lastUserInput = now;
      if (uiPage == 0) uiPage = (UiPage)(PAGE_COUNT - 1);
      else uiPage = (UiPage)(uiPage - 1);
    } else if (joyY < 260) {
      moved = true;
      lastJoyAction = now;
      lastUserInput = now;
      uiPage = (UiPage)((uiPage + 1) % PAGE_COUNT);
    }
  }

  if (menuActive && !moved && (now - lastJoyAction >= JOYSTICK_REPEAT_MS)) {
    if (joyX > 760 || joyX < 260) {
      int delta = (joyX > 760) ? 1 : -1;
      lastJoyAction = now;
      lastUserInput = now;

      if (uiPage == PAGE_FLAME) {
        cfg.flameTrigger = clampInt(cfg.flameTrigger + delta * 10, 50, 950);
      } else if (uiPage == PAGE_FAN) {
        int idx = fanConfigIndex;
        int updated = (int)cfg.fanDutyByLevel[idx] + delta * 5;
        cfg.fanDutyByLevel[idx] = (uint8_t)clampInt(updated, 0, 100);
      }
    }
  }

  if (menuActive && (now - lastUserInput >= MENU_IDLE_TIMEOUT_MS)) {
    menuActive = false;
  }
}

void updateStateTask(unsigned long now) {
  if (now - lastStateUpdate < STATE_MS) return;
  lastStateUpdate = now;

  AirLevel mqLevel = levelFromThresholds(sensor.mqRaw, cfg.mqYellow, cfg.mqOrange, cfg.mqRed);

  AirLevel pm25Level = AIR_GREEN;
  AirLevel pm10Level = AIR_GREEN;
  if (sensor.pmValid) {
    pm25Level = levelFromThresholds((int)sensor.pm.pm25_standard,
                                    cfg.pm25Yellow, cfg.pm25Orange, cfg.pm25Red);
    pm10Level = levelFromThresholds((int)sensor.pm.pm100_standard,
                                    cfg.pm10Yellow, cfg.pm10Orange, cfg.pm10Red);
  }

  airLevel = mqLevel;
  if ((int)pm25Level > (int)airLevel) airLevel = pm25Level;
  if ((int)pm10Level > (int)airLevel) airLevel = pm10Level;

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
  uint16_t blinkPeriod = 0;
  if (systemState == ALERT_ORANGE) blinkPeriod = 500;
  if (systemState == ALERT_RED) blinkPeriod = 250;
  if (systemState == FIRE_ALERT) blinkPeriod = 120;

  if (blinkPeriod > 0 && now - blinkMark >= blinkPeriod) {
    blinkMark = now;
    blinkOn = !blinkOn;
  } else if (blinkPeriod == 0) {
    blinkOn = true;
  }

  if (!blinkOn && blinkPeriod > 0) {
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
  static bool chirpToggle = false;

  if (systemState == FIRE_ALERT) {
    if (now - lastTone >= 180) {
      lastTone = now;
      chirpToggle = !chirpToggle;
      tone(BUZZER_PIN, chirpToggle ? 2000 : 1400, 140);
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
  if (now - lastOutputUpdate < OUTPUT_MS) return;
  lastOutputUpdate = now;

  applyLedPattern(now);
  applyBuzzerPattern(now);
  applyFanControl();
}

void renderIdleClock(unsigned long now) {
  unsigned long sec = now / 1000UL;
  uint8_t hh = (sec / 3600UL) % 24;
  uint8_t mm = (sec / 60UL) % 60;
  uint8_t ss = sec % 60;

  lcd.setCursor(0, 0);
  lcd.print("Clock ");
  if (hh < 10) lcd.print('0');
  lcd.print(hh);
  lcd.print(':');
  if (mm < 10) lcd.print('0');
  lcd.print(mm);
  lcd.print(':');
  if (ss < 10) lcd.print('0');
  lcd.print(ss);
  lcd.print("   ");

  lcd.setCursor(0, 1);
  lcd.print("State:");
  lcd.print(levelName(airLevel));
  lcd.print("       ");

  lcd.setCursor(0, 2);
  lcd.print("Gas:");
  lcd.print(sensor.mqRaw);
  lcd.print(" PM2.5:");
  if (sensor.pmValid) lcd.print(sensor.pm.pm25_standard);
  else lcd.print("--");
  lcd.print("   ");

  lcd.setCursor(0, 3);
  lcd.print("Flame:");
  lcd.print(sensor.flameAvg);
  lcd.print(sensor.flameDetected ? " FIRE " : " SAFE ");
}

void renderGasPage() {
  lcd.setCursor(0, 0);
  lcd.print("Gas Sensor (MQ-135)  ");
  lcd.setCursor(0, 1);
  lcd.print("Raw ADC: ");
  lcd.print(sensor.mqRaw);
  lcd.print("       ");
  lcd.setCursor(0, 2);
  lcd.print("Est ppm: ");
  lcd.print(sensor.gasPpmEst);
  lcd.print(" (10-1000)");
  lcd.setCursor(0, 3);
  lcd.print("Lvl:");
  lcd.print(levelName(airLevel));
  lcd.print(" Btn=Exit");
}

void renderPmPage() {
  lcd.setCursor(0, 0);
  lcd.print("PM1 PM2.5 PM10       ");
  if (sensor.pmValid) {
    lcd.setCursor(0, 1);
    lcd.print(sensor.pm.pm10_standard);
    lcd.print("   ");
    lcd.print(sensor.pm.pm25_standard);
    lcd.print("   ");
    lcd.print(sensor.pm.pm100_standard);
    lcd.print("   ");
    lcd.setCursor(0, 2);
    lcd.print("0.3:");
    lcd.print(sensor.pm.particles_03um);
    lcd.print(" 1.0:");
    lcd.print(sensor.pm.particles_10um);
    lcd.print("   ");
    lcd.setCursor(0, 3);
    lcd.print("2.5:");
    lcd.print(sensor.pm.particles_25um);
    lcd.print(" 5.0:");
    lcd.print(sensor.pm.particles_50um);
    lcd.print("   ");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("PM sensor read failed");
    lcd.setCursor(0, 2);
    lcd.print("Check wiring on D2/D3");
    lcd.setCursor(0, 3);
    lcd.print("Btn=Exit  Up/Down nav");
  }
}

void renderTempHumPage() {
  lcd.setCursor(0, 0);
  lcd.print("Temp / Humidity      ");
  if (sensor.dhtValid) {
    float tempF = sensor.tempC * 9.0 / 5.0 + 32.0;
    lcd.setCursor(0, 1);
    lcd.print("T: ");
    lcd.print(sensor.tempC, 1);
    lcd.print("C  ");
    lcd.print(tempF, 1);
    lcd.print("F   ");
    lcd.setCursor(0, 2);
    lcd.print("RH: ");
    lcd.print(sensor.humidity, 1);
    lcd.print("%              ");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("DHT22 read error     ");
    lcd.setCursor(0, 2);
    lcd.print("Check D4 + 5V/GND    ");
  }
  lcd.setCursor(0, 3);
  lcd.print("Up/Down nav Btn=Exit ");
}

void renderFlamePage() {
  lcd.setCursor(0, 0);
  lcd.print("Flame Sensor (KY-026)");
  lcd.setCursor(0, 1);
  lcd.print("Raw:");
  lcd.print(sensor.flameRaw);
  lcd.print(" Avg:");
  lcd.print(sensor.flameAvg);
  lcd.print("   ");
  lcd.setCursor(0, 2);
  lcd.print("Trig:");
  lcd.print(cfg.flameTrigger);
  lcd.print(" Hys:");
  lcd.print(cfg.flameHysteresis);
  lcd.print("   ");
  lcd.setCursor(0, 3);
  lcd.print("L/R trig ");
  lcd.print(sensor.flameDetected ? "FIRE" : "SAFE");
  lcd.print(" ");
}

void renderFanPage() {
  const char* names[4] = {"G", "Y", "O", "R"};
  lcd.setCursor(0, 0);
  lcd.print("Fan Duty by Level    ");
  lcd.setCursor(0, 1);
  lcd.print("G:");
  lcd.print((int)cfg.fanDutyByLevel[0]);
  lcd.print("% Y:");
  lcd.print((int)cfg.fanDutyByLevel[1]);
  lcd.print("% O:");
  lcd.print((int)cfg.fanDutyByLevel[2]);
  lcd.print("%");
  lcd.setCursor(0, 2);
  lcd.print("R:");
  lcd.print((int)cfg.fanDutyByLevel[3]);
  lcd.print("% Now:");
  lcd.print(fanDuty);
  lcd.print("%   ");
  lcd.setCursor(0, 3);
  lcd.print("Sel:");
  lcd.print(names[fanConfigIndex]);
  lcd.print(" L/R adj Btn=Next/Ex");
}

void updateUiTask(unsigned long now) {
  if (now - lastUiUpdate < UI_MS) return;
  lastUiUpdate = now;

  if (!menuActive && systemState != FIRE_ALERT && systemState != ALERT_ORANGE && systemState != ALERT_RED) {
    renderIdleClock(now);
    return;
  }

  if (systemState == FIRE_ALERT && !menuActive) {
    lcd.setCursor(0, 0);
    lcd.print("!!! FIRE ALERT !!!   ");
    lcd.setCursor(0, 1);
    lcd.print("Flame Avg:");
    lcd.print(sensor.flameAvg);
    lcd.print("      ");
    lcd.setCursor(0, 2);
    lcd.print("Fan forced 100%      ");
    lcd.setCursor(0, 3);
    lcd.print("Press btn -> menu    ");
    return;
  }

  if (!menuActive && (systemState == ALERT_ORANGE || systemState == ALERT_RED)) {
    lcd.setCursor(0, 0);
    lcd.print("Air Warning Active   ");
    lcd.setCursor(0, 1);
    lcd.print("Level: ");
    lcd.print(levelName(airLevel));
    lcd.print("          ");
    lcd.setCursor(0, 2);
    lcd.print("Gas:");
    lcd.print(sensor.mqRaw);
    lcd.print(" PM2.5:");
    if (sensor.pmValid) lcd.print(sensor.pm.pm25_standard);
    else lcd.print("--");
    lcd.print("   ");
    lcd.setCursor(0, 3);
    lcd.print("Press btn for menu   ");
    return;
  }

  if (uiPage == PAGE_GAS) renderGasPage();
  if (uiPage == PAGE_PM) renderPmPage();
  if (uiPage == PAGE_TEMP_HUMID) renderTempHumPage();
  if (uiPage == PAGE_FLAME) renderFlamePage();
  if (uiPage == PAGE_FAN) renderFanPage();
}

void serialTelemetryTask(unsigned long now) {
  if (now - lastSerialUpdate < SERIAL_MS) return;
  lastSerialUpdate = now;

  // Includes your requested flame demo behavior (analog read print cadence),
  // integrated into full telemetry.
  Serial.println("---- Home Air Monitor ----");
  Serial.print("Flame analog (A3): ");
  Serial.println(sensor.flameRaw);
  Serial.print("Flame avg: ");
  Serial.print(sensor.flameAvg);
  Serial.print(" | Flame detected: ");
  Serial.println(sensor.flameDetected ? "YES" : "NO");

  Serial.print("MQ135 raw: ");
  Serial.print(sensor.mqRaw);
  Serial.print(" | Gas est ppm: ");
  Serial.println(sensor.gasPpmEst);

  if (sensor.pmValid) {
    Serial.print("PM1.0: ");
    Serial.print(sensor.pm.pm10_standard);
    Serial.print(" | PM2.5: ");
    Serial.print(sensor.pm.pm25_standard);
    Serial.print(" | PM10: ");
    Serial.println(sensor.pm.pm100_standard);
  } else {
    Serial.println("PM25: read error");
  }

  if (sensor.dhtValid) {
    Serial.print("Temp C: ");
    Serial.print(sensor.tempC, 1);
    Serial.print(" | Humidity %: ");
    Serial.println(sensor.humidity, 1);
  } else {
    Serial.println("DHT22: read error");
  }

  Serial.print("State: ");
  if (systemState == IDLE_CLOCK) Serial.print("IDLE_CLOCK");
  if (systemState == MENU) Serial.print("MENU");
  if (systemState == ALERT_ORANGE) Serial.print("ALERT_ORANGE");
  if (systemState == ALERT_RED) Serial.print("ALERT_RED");
  if (systemState == FIRE_ALERT) Serial.print("FIRE_ALERT");
  Serial.print(" | Air level: ");
  Serial.print(levelName(airLevel));
  Serial.print(" | Fan duty: ");
  Serial.println(fanDuty);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Boot: Serial begin complete");

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  Serial.println("Boot: pinMode setup complete");

  setColor(0, 0, 0);
  noTone(BUZZER_PIN);
  Serial.println("Boot: LED/Buzzer initialized");

  Serial.println("Boot: LCD init start");
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PHYS124 Air System");
  lcd.setCursor(0, 1);
  lcd.print("Init sensors...");
  Serial.println("Boot: LCD init complete");

  Serial.println("Boot: DHT begin start");
  dht.begin();
  Serial.println("Boot: DHT begin complete");

  Serial.println("Boot: PM serial begin start");
  pmSerial.begin(9600);
  Serial.println("Boot: PM warmup delay start");
  delay(1500);
  Serial.println("Boot: PM begin_UART start");
  pmReady = aqi.begin_UART(&pmSerial);
  Serial.println(pmReady ? "Boot: PM begin_UART success" : "Boot: PM begin_UART failed");

  Serial.println("Boot: Fan setup start");
  setupFan25kHz();
  setFanDuty(cfg.fanDutyByLevel[0]);
  Serial.println("Boot: Fan setup complete");

  Serial.println("Boot: Flame baseline read");
  sensor.flameRaw = analogRead(FLAME_PIN);
  updateFlameFilter(sensor.flameRaw);
  Serial.print("Boot: Flame baseline raw = ");
  Serial.println(sensor.flameRaw);

  lcd.setCursor(0, 2);
  lcd.print(pmReady ? "PM5003 online       " : "PM5003 not detected ");
  lcd.setCursor(0, 3);
  lcd.print("Press joystick btn  ");
  delay(1000);
  lcd.clear();

  Serial.println("Home Air Quality Monitor started.");
}

void loop() {
  unsigned long now = millis();
  readSensorsTask(now);
  handleInputTask(now);
  updateStateTask(now);
  updateOutputTask(now);
  updateUiTask(now);
  serialTelemetryTask(now);

  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    Serial.print("Heartbeat ms=");
    Serial.print(now);
    Serial.print(" state=");
    Serial.print((int)systemState);
    Serial.print(" menu=");
    Serial.print(menuActive ? 1 : 0);
    Serial.print(" pmReady=");
    Serial.print(pmReady ? 1 : 0);
    Serial.print(" pmValid=");
    Serial.println(sensor.pmValid ? 1 : 0);
  }
}
