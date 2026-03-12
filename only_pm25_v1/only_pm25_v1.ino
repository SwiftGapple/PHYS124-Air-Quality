/* Test sketch for Adafruit PM2.5 sensor with UART or I2C */

#include "Adafruit_PM25AQI.h"
// pin #2 is IN from sensor (TX pin on sensor), leave pin #3 disconnected
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
SoftwareSerial pmSerial(2, 3);

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit PMSA003I Air Quality Sensor");

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PM Sensor Startup");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  pmSerial.begin(9600);

  // Wait three seconds for sensor to boot up!
  delay(3000);


  if (! aqi.begin_UART(&pmSerial)) { // connect to the sensor over software serial 
    Serial.println("Could not find PM 2.5 sensor!");
    lcd.setCursor(0, 2);
    lcd.print("PM sensor not found");
    while (1) delay(10);
  }

  Serial.println("PM25 found!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PM25 Ready");
}

void loop() {
  PM25_AQI_Data data;
  
  if (! aqi.read(&data)) {
    Serial.println("Could not read from AQI");
    lcd.setCursor(0, 0); lcd.print("PM read failed       ");
    lcd.setCursor(0, 1); lcd.print("Retrying...          ");
    lcd.setCursor(0, 2); lcd.print("                    ");
    lcd.setCursor(0, 3); lcd.print("                    ");
    
    delay(500);  // try again in a bit!
    return;
  }
  Serial.println("AQI reading success");

  Serial.println(F("---------------------------------------"));
  Serial.println(F("Concentration Units (standard)"));
  Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
  Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Concentration Units (environmental)"));
  Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_env);
  Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_env);
  Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_env);
  Serial.println(F("---------------------------------------"));
  Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(data.particles_03um);
  Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
  Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
  Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
  Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
  Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
  Serial.println(F("---------------------------------------"));
  Serial.println(F("AQI"));
  Serial.print(F("PM2.5 AQI US: ")); Serial.print(data.aqi_pm25_us);
  Serial.print(F("\tPM10  AQI US: ")); Serial.println(data.aqi_pm100_us);
//  Serial.print(F("PM2.5 AQI China: ")); Serial.print(data.aqi_pm25_china);
//  Serial.print(F("\tPM10  AQI China: ")); Serial.println(data.aqi_pm100_china);
  Serial.println(F("---------------------------------------"));
  Serial.println();

  // LCD real-time PM display (standard concentration)
  lcd.setCursor(0, 0); lcd.print("PMS5003 Real-time    ");
  lcd.setCursor(0, 1); lcd.print("PM1.0: "); lcd.print(data.pm10_standard); lcd.print("      ");
  lcd.setCursor(0, 2); lcd.print("PM2.5: "); lcd.print(data.pm25_standard); lcd.print("      ");
  lcd.setCursor(0, 3); lcd.print("PM10 : "); lcd.print(data.pm100_standard); lcd.print("      ");

  delay(1000);
}