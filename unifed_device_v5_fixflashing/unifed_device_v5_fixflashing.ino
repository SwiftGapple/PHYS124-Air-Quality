#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <stdarg.h>

// Operating principle:
// 1) Periodic tasks read sensors, process state, drive outputs, and update UI.
// 2) Gas/flame levels map to warning states that control RGB LED, buzzer, and fan.
// 3) LCD pages are joystick-driven, and cached line rendering avoids flicker.

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
#define FAN_PIN 9

// ---- Timing ----
const unsigned long FAST_SENSOR_MS = 120;
const unsigned long SLOW_SENSOR_MS = 1200;
const unsigned long STATE_MS = 160;
const unsigned long OUTPUT_MS = 80;
const unsigned long UI_MS = 220;
const unsigned long JOY_REPEAT_MS = 220;
const unsigned long MENU_TIMEOUT_MS = 200000;

const uint8_t FLAME_FILTER_N = 8;
const int JOY_LOW = 260;
const int JOY_HIGH = 760;

LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPIN, DHTTYPE);

int icr = 639; // Timer1 TOP for ~25 kHz
int fanDuty = 10;

enum AirLevel { AIR_GREEN = 0, AIR_YELLOW = 1, AIR_ORANGE = 2, AIR_RED = 3 };
enum SystemState { IDLE_CLOCK = 0, MENU = 1, ALERT_ORANGE = 2, ALERT_RED = 3, FIRE_ALERT = 4 };
enum UiPage { PAGE_GAS = 0, PAGE_TEMP = 1, PAGE_FLAME = 2, PAGE_FAN = 3, PAGE_CLOCK = 4, PAGE_COUNT = 5 };

struct SensorData {
  int mqRaw;
  uint16_t gasPpmEst;
  int flameRaw;
  int flameAvg;
  bool flameDetected;
  float tempC;
  float humidity;
  bool dhtValid;
};

struct ThresholdConfig {
  int mqYellow;
  int mqOrange;
  int mqRed;
  int flameYellow;
  int flameOrange;
  int flameRed;
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

SensorData sensor = {0, 10, 0, 0, false, NAN, NAN, false};
ThresholdConfig cfg = {
  120, 160, 200, // MQ thresholds tuned for 20..300 raw range
  950, 900, 860, // Flame warning thresholds (lower value = stronger flame)
  820, 30,       // Flame fire trigger + hysteresis
  {20, 35, 65, 90}
};
ClockData clockData = {12, 0, 0, false, 0};

SystemState systemState = IDLE_CLOCK;
AirLevel airLevel = AIR_GREEN;
UiPage uiPage = PAGE_GAS;
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
char lcdCache[4][21];

// Build a fixed-width 20-char LCD line from printf-style input.
void formatLcdLine(char* out, const char* fmt, ...) {
  char buf[48];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  uint8_t i = 0;
  for (; i < 20 && buf[i] != '\0'; i++) out[i] = buf[i];
  for (; i < 20; i++) out[i] = ' ';
  out[20] = '\0';
}

// Write a row only when changed, reducing I2C traffic and flicker.
void writeLineIfChanged(uint8_t row, const char* line) {
  if (strcmp(lcdCache[row], line) == 0) return; // Skip unchanged row.
  lcd.setCursor(0, row);
  lcd.print(line);
  strcpy(lcdCache[row], line);
}

// Render all four rows using line-level change detection.
void renderLines(const char* l0, const char* l1, const char* l2, const char* l3) {
  writeLineIfChanged(0, l0);
  writeLineIfChanged(1, l1);
  writeLineIfChanged(2, l2);
  writeLineIfChanged(3, l3);
}

// Set RGB LED PWM outputs.
void setColor(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED, r);
  analogWrite(GREEN, g);
  analogWrite(BLUE, b);
}

// Configure Timer1 for fan PWM generation.
void setupFan25kHz() {
  pinMode(FAN_PIN, OUTPUT);
  TCCR1A = 0xA2;
  TCCR1B = 0x19;
  TCNT1H = 0x00;
  TCNT1L = 0x00;
  ICR1H = icr >> 8;
  ICR1L = icr & 0x00ff;
}

// Apply fan duty percentage and update PWM compare value.
void setFanDuty(int duty) {
  if (duty < 0) duty = 0;
  if (duty > 100) duty = 100;
  fanDuty = duty;
  OCR1A = (uint16_t)(((long)icr * fanDuty) / 100L);
}

// Convert MQ raw value to display-only ppm estimate.
uint16_t estimatePpmFromMq(int raw) {
  // MQ sensitivity range tuned for this setup.
  long ppm = map(raw, 20, 300, 10, 1000);
  if (ppm < 10) ppm = 10;
  if (ppm > 1000) ppm = 1000;
  return (uint16_t)ppm;
}

// Classify a "higher is worse" value into air levels.
AirLevel levelFromThresholds(int value, int yellow, int orange, int red) {
  if (value >= red) return AIR_RED;
  if (value >= orange) return AIR_ORANGE;
  if (value >= yellow) return AIR_YELLOW;
  return AIR_GREEN;
}

// Classify a "lower is worse" value into air levels.
AirLevel levelFromLowerIsWorse(int value, int yellow, int orange, int red) {
  if (value <= red) return AIR_RED;
  if (value <= orange) return AIR_ORANGE;
  if (value <= yellow) return AIR_YELLOW;
  return AIR_GREEN;
}

// Return text label for air level.
const char* levelName(AirLevel l) {
  if (l == AIR_GREEN) return "GREEN";
  if (l == AIR_YELLOW) return "YELLOW";
  if (l == AIR_ORANGE) return "ORANGE";
  return "RED";
}

// Update moving-average filter for flame analog signal.
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

// Latch/unlatch flame fire detection with hysteresis.
void updateFlameDetection() {
  // KY-026 analog typically decreases when flame IR increases.
  if (!sensor.flameDetected && sensor.flameAvg <= cfg.flameTrigger) {
    sensor.flameDetected = true;
  } else if (sensor.flameDetected &&
             sensor.flameAvg >= (cfg.flameTrigger + cfg.flameHysteresis)) {
    sensor.flameDetected = false;
  }
}

// Increment software clock from millis().
void tickClock(unsigned long now) {
  if (now - lastTraceTickClock >= 2000UL) {
    lastTraceTickClock = now;
    Serial.println(F("[TASK] tickClock"));
  }

  while (now - lastClockTick >= 1000UL) { // Catch up if loop was delayed.
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

// Adjust selected clock field (HH/MM/SS) with wraparound.
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

// Periodically sample sensors and input devices.
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

  }
}

// Process joystick button and axis events for menu navigation/editing.
void handleInputTask(unsigned long now) {
  if (now - lastTraceInput >= 2000UL) {
    lastTraceInput = now;
    Serial.println(F("[TASK] handleInputTask"));
  }

  bool btnEvent = joyPressed && !prevJoyPressed; // Rising edge detect.
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
  if (now - lastInputTick < JOY_REPEAT_MS) return; // Debounce + repeat rate.

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

// Derive system state and dominant alert level from sensor values.
void updateStateTask(unsigned long now) {
  if (now - lastTraceState >= 2000UL) {
    lastTraceState = now;
    Serial.println(F("[TASK] updateStateTask"));
  }

  if (now - lastStateTick < STATE_MS) return;
  lastStateTick = now;

  AirLevel mqLevel = levelFromThresholds(sensor.mqRaw, cfg.mqYellow, cfg.mqOrange, cfg.mqRed);
  AirLevel flameLevel = levelFromLowerIsWorse(sensor.flameAvg, cfg.flameYellow, cfg.flameOrange, cfg.flameRed);

  airLevel = mqLevel;
  outlierValue = sensor.mqRaw;
  if ((int)flameLevel > (int)airLevel) { // Use the worst (highest severity).
    airLevel = flameLevel;
    outlierValue = sensor.flameAvg;
  }

  if (sensor.flameDetected) { // Fire state overrides all normal warnings.
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

// Apply LED color/blink pattern based on state and severity.
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

  if (!blinkOn && period > 0) { // Blink-off phase.
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

// Apply buzzer cadence/tone pattern for warning and fire states.
void applyBuzzerPattern(unsigned long now) {
  static unsigned long lastTone = 0;
  static bool fireFlip = false;

  if (systemState == FIRE_ALERT) { // Fast alternating alarm for fire.
    if (now - lastTone >= 180) {
      lastTone = now;
      fireFlip = !fireFlip;
      tone(BUZZER_PIN, fireFlip ? 2000 : 1400, 140);
    }
    return;
  }
  if (systemState == ALERT_RED) { // Strong periodic warning.
    if (now - lastTone >= 500) {
      lastTone = now;
      tone(BUZZER_PIN, 1300, 220);
    }
    return;
  }
  if (systemState == ALERT_ORANGE) { // Softer periodic warning.
    if (now - lastTone >= 900) {
      lastTone = now;
      tone(BUZZER_PIN, 900, 150);
    }
    return;
  }
  noTone(BUZZER_PIN);
}

// Apply automatic fan duty from current severity/state.
void applyFanControl() {
  if (systemState == FIRE_ALERT) {
    setFanDuty(100);
    return;
  }
  setFanDuty(cfg.fanDutyByLevel[(int)airLevel]);
}

// Periodic output task for LED, buzzer, and fan.
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

// Render idle clock/home page.
void renderIdleClock() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "Clock %02u:%02u:%02u", clockData.hh, clockData.mm, clockData.ss);
  formatLcdLine(l1, "Air:%s Fan:%d%%", levelName(airLevel), fanDuty);
  formatLcdLine(l2, "Outlier:Gas %d", outlierValue);
  formatLcdLine(l3, "Btn=Menu AutoAlertOn");
  renderLines(l0, l1, l2, l3);
}

// Render hazardous gas page.
void renderGasPage() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "Gas Sensor MQ-135");
  formatLcdLine(l1, "Raw ADC: %d", sensor.mqRaw);
  formatLcdLine(l2, "Est ppm: %u 10-1000", sensor.gasPpmEst);
  formatLcdLine(l3, "Lvl:%s Btn=Exit", levelName(airLevel));
  renderLines(l0, l1, l2, l3);
}

// Render temperature and humidity page.
void renderTempPage() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "Temperature/Humidity");
  if (!sensor.dhtValid) {
    formatLcdLine(l1, "DHT22 read error");
    formatLcdLine(l2, "Check D4/5V/GND");
    formatLcdLine(l3, "Y nav Btn=Exit");
    renderLines(l0, l1, l2, l3);
    return;
  }

  // Avoid printf float formatting on Arduino cores (often disabled),
  // so line text changes correctly and cache updates work.
  int tempC10 = (int)(sensor.tempC * 10.0f + (sensor.tempC >= 0 ? 0.5f : -0.5f));
  int tempF10 = (tempC10 * 9) / 5 + 320;
  int hum10 = (int)(sensor.humidity * 10.0f + (sensor.humidity >= 0 ? 0.5f : -0.5f));
  int tempCAbs = tempC10 < 0 ? -tempC10 : tempC10;
  int tempFAbs = tempF10 < 0 ? -tempF10 : tempF10;
  int humAbs = hum10 < 0 ? -hum10 : hum10;

  formatLcdLine(l1, "T:%s%d.%dC %s%d.%dF",
                tempC10 < 0 ? "-" : "",
                tempCAbs / 10, tempCAbs % 10,
                tempF10 < 0 ? "-" : "",
                tempFAbs / 10, tempFAbs % 10);
  formatLcdLine(l2, "RH:%s%d.%d%%",
                hum10 < 0 ? "-" : "",
                humAbs / 10, humAbs % 10);
  formatLcdLine(l3, "Y nav Btn=Exit");
  renderLines(l0, l1, l2, l3);
}

// Render flame status page and thresholds.
void renderFlamePage() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "Flame Sensor KY-026");
  formatLcdLine(l1, "Raw:%d Avg:%d", sensor.flameRaw, sensor.flameAvg);
  formatLcdLine(l2, "Y:%d O:%d R:%d", cfg.flameYellow, cfg.flameOrange, cfg.flameRed);
  formatLcdLine(l3, "%s FIRE@%d Btn=Exit", sensor.flameDetected ? "FIRE" : "SAFE", cfg.flameTrigger);
  renderLines(l0, l1, l2, l3);
}

// Render fan configuration/active duty page.
void renderFanPage() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "Fan Config by Level");
  formatLcdLine(l1, "G:%d%% Y:%d%% O:%d%%", (int)cfg.fanDutyByLevel[0], (int)cfg.fanDutyByLevel[1], (int)cfg.fanDutyByLevel[2]);
  formatLcdLine(l2, "R:%d%% Run:%d%%", (int)cfg.fanDutyByLevel[3], fanDuty);
  formatLcdLine(l3, "Auto control (locked)");
  renderLines(l0, l1, l2, l3);
}

// Render clock setup page.
void renderClockPage() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "Clock Setup");
  formatLcdLine(l1, "Time %02u:%02u:%02u", clockData.hh, clockData.mm, clockData.ss);
  if (!clockData.editMode) {
    formatLcdLine(l2, "Btn=Edit  Y nav");
  } else {
    const char* field = (clockData.field == 0) ? "HH" : (clockData.field == 1) ? "MM" : "SS";
    formatLcdLine(l2, "L/R adjust %s", field);
  }
  formatLcdLine(l3, "Btn step/save+exit");
  renderLines(l0, l1, l2, l3);
}

// Render orange/red warning summary page.
void renderAlertScreen() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "Air Warning Active");
  formatLcdLine(l1, "Level: %s", levelName(airLevel));
  formatLcdLine(l2, "Outlier: Gas %d", outlierValue);
  formatLcdLine(l3, "Auto fan+alarm BtnUI");
  renderLines(l0, l1, l2, l3);
}

// Render fire-priority alert page.
void renderFireScreen() {
  char l0[21], l1[21], l2[21], l3[21];
  formatLcdLine(l0, "!!! FIRE ALERT !!!");
  formatLcdLine(l1, "Flame avg: %d", sensor.flameAvg);
  formatLcdLine(l2, "Fan forced to 100%%");
  formatLcdLine(l3, "Btn=Menu");
  renderLines(l0, l1, l2, l3);
}

// Periodic UI task that selects and renders the active screen.
void updateUiTask(unsigned long now) {
  if (now - lastTraceUi >= 2000UL) {
    lastTraceUi = now;
    Serial.println(F("[TASK] updateUiTask"));
  }

  if (now - lastUiTick < UI_MS) return; // UI frame limiter.
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
  if (uiPage == PAGE_TEMP) renderTempPage();
  if (uiPage == PAGE_FLAME) renderFlamePage();
  if (uiPage == PAGE_FAN) renderFanPage();
  if (uiPage == PAGE_CLOCK) renderClockPage();
}

// Initialize peripherals, sensors, and initial UI.
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
  for (uint8_t i = 0; i < 4; i++) lcdCache[i][0] = '\0';
  lcd.setCursor(0, 0); lcd.print("PHYS124 Air Monitor  ");
  lcd.setCursor(0, 1); lcd.print("Sensor startup...    ");

  dht.begin();
  delay(500);

  setupFan25kHz();
  setFanDuty(cfg.fanDutyByLevel[0]);

  sensor.flameRaw = analogRead(FLAME_PIN);
  updateFlameFilter(sensor.flameRaw);

  lastClockTick = millis();
  lastUserInput = lastClockTick;
  lcd.setCursor(0, 2); lcd.print("PM sensor removed    ");
  lcd.setCursor(0, 3); lcd.print("Press joy btn for UI ");
  delay(3000);
  lcd.clear();
}

// Main cooperative loop scheduler.
void loop() {
  unsigned long now = millis();
  tickClock(now);
  readSensorsTask(now);
  handleInputTask(now);
  updateStateTask(now);
  updateOutputTask(now);
  updateUiTask(now);
}
