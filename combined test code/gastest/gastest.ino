#define MQ135_PIN A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  int gasValue = analogRead(MQ135_PIN);

  Serial.print("MQ-135 Analog Value: ");
  Serial.println(gasValue);

  delay(1000);
}
