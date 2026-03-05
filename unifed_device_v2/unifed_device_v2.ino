#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include "Adafruit_PM25AQI.h"

// ---- Pin mappings from original sample sketches ----
#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define JOY_BTN_PIN 7

#define MQ135_PIN A2

define FLAME_PIN = A3

#define DHTPIN 4
#define DHTTYPE DHT22

#define BUZZER_PIN 5

#define RED 6
#define GREEN 10
#define BLUE 11

#define PM25_RX_PIN 2   // sensor TX -> Arduino pin 2
#define PM25_TX_PIN 3   // unused for sensor RX in this sample wiring

#define FAN_PIN 9

// ---- Module objects ----
LiquidCrystal_I2C lcd(0x27, 20, 4);
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial pmSerial(PM25_RX_PIN, PM25_TX_PIN);
Adafruit_PM25AQI aqi;

// ---- PWM fan setup variables from sample ----
int icr = 639;
int duty_cycle = 80;
int fanDuty = 10;

// ---- Runtime state ----
bool pmReady = false;
unsigned long lastSensorPrint = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastLedUpdate = 0;
unsigned long lastFanUpdate = 0;
unsigned long lastBuzzerUpdate = 0;

int gasValue = 0;
int joyX = 0;
int joyY = 0;
bool joyPressed = false;
float humidity = NAN;
float temperature = NAN;
PM25_AQI_Data pmData;
bool pmDataValid = false;

int ledIndex = 0;
int buzzerIndex = 0;
const int buzzerNotes[] = {523, 659, 784};
const int buzzerCount = sizeof(buzzerNotes) / sizeof(buzzerNotes[0]);

void setColor(int r, int g, int b) {
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
  OCR1A = icr * (duty_cycle / 100.0);
}

void setFanDuty(int duty) {
  fanDuty = duty;
  OCR1A = icr * (fanDuty / 100.0);
}

void setup() {
  Serial.begin(9600);

  // LED
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  setColor(0, 0, 0);

  // Joystick button
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);

  // Passive buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Combined Sensor Test");

  // DHT22
  dht.begin();

  // PM2.5
  pmSerial.begin(9600);
  delay(3000); // keep original PM25 startup delay
  pmReady = aqi.begin_UART(&pmSerial);
  if (pmReady) {
    Serial.println("PM25 found!");
  } else {
    Serial.println("Could not find PM 2.5 sensor!");
  }

  // PWM fan
  setupFan25kHz();
  setFanDuty(10);
  Serial.println("Combined test started.");

  // Flame sensor
  sensor.flameRaw = analogRead(FLAME_PIN);
  updateFlameFilter(sensor.flameRaw);
  Serial.print("Flame baseline raw = ");
  Serial.println(sensor.flameRaw);

  delay(3000);

  lcd.clear();
}

void loop() {
  unsigned long now = millis();

  // LED color cycle
  if (now - lastLedUpdate >= 1000) {
    lastLedUpdate = now;
    if (ledIndex == 0) setColor(255, 0, 0);
    if (ledIndex == 1) setColor(0, 255, 0);
    if (ledIndex == 2) setColor(0, 0, 255);
    if (ledIndex == 3) setColor(255, 0, 255);
    ledIndex = (ledIndex + 1) % 4;
  }

  // Passive buzzer tones
  if (now - lastBuzzerUpdate >= 2000) {
    lastBuzzerUpdate = now;
    tone(BUZZER_PIN, buzzerNotes[buzzerIndex], 250);
    buzzerIndex = (buzzerIndex + 1) % buzzerCount;
  }

  // Fan duty alternates 10% and 90%
  if (now - lastFanUpdate >= 5000) {
    lastFanUpdate = now;
    setFanDuty(fanDuty == 10 ? 90 : 10);
  }

  // Read and print sensors
  if (now - lastSensorPrint >= 1000) {
    lastSensorPrint = now;

    gasValue = analogRead(MQ135_PIN);
    joyX = analogRead(JOY_X_PIN);
    joyY = analogRead(JOY_Y_PIN);
    joyPressed = (digitalRead(JOY_BTN_PIN) == LOW);

    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    if (pmReady) {
      pmDataValid = aqi.read(&pmData);
    } else {
      pmDataValid = false;
    }

    Serial.println("---- Combined Test Readings ----");
    Serial.print("Hazardous Gas Level (MQ135): ");
    Serial.println(gasValue);

    Serial.print("Joystick X: ");
    Serial.print(joyX);
    Serial.print(" | Y: ");
    Serial.print(joyY);
    Serial.print(" | Button Pressed: ");
    Serial.println(joyPressed ? "YES" : "NO");

    Serial.print("Fan Duty cycle (%) = ");
    Serial.println(fanDuty);

    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Temp/Humidity: DHT read error");
    } else {
      Serial.print("Temp: ");
      Serial.print(temperature);
      Serial.print(" C | Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
    }

    if (pmDataValid) {
      Serial.print("PM1.0 std: ");
      Serial.print(pmData.pm10_standard);
      Serial.print(" | PM2.5 std: ");
      Serial.print(pmData.pm25_standard);
      Serial.print(" | PM10 std: ");
      Serial.println(pmData.pm100_standard);

      Serial.print("PM2.5 AQI US: ");
      Serial.print(pmData.aqi_pm25_us);
      Serial.print(" | PM10 AQI US: ");
      Serial.println(pmData.aqi_pm100_us);

      Serial.println("-------------------------");
    } else {
      Serial.println("PM25: Could not read from AQI");
    }
  }

  // LCD constantly changing display
  if (now - lastLcdUpdate >= 500) {
    lastLcdUpdate = now;
    lcd.setCursor(0, 0);
    lcd.print("Time Passed: ");
    lcd.print(now / 1000);
    lcd.print("s");

    lcd.setCursor(0, 1);
    lcd.print("Gas:");
    lcd.print(gasValue);
    lcd.print(" Joy:");
    lcd.print(joyPressed ? "P" : "R");
    lcd.print("   ");

    lcd.setCursor(0, 2);
    lcd.print("X:");
    lcd.print(joyX);
    lcd.print(" Y:");
    lcd.print(joyY);
    lcd.print("    ");

    lcd.setCursor(0, 3);
    if (isnan(humidity) || isnan(temperature)) {
      lcd.print("Temp/Humi read err  ");
    } else {
      lcd.print("T:");
      lcd.print(temperature, 1);
      lcd.print("C H:");
      lcd.print(humidity, 1);
      lcd.print("%   ");
    }
  }
}
