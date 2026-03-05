int icr, frecuency, duty_cycle;

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  
  // Timer/Counter 1 initialization
  TCCR1A = 0xA2;
  TCCR1B = 0x19;
  TCNT1H = 0x00;
  TCNT1L = 0x00;

  duty_cycle = 80; // Change this value to the desired duty cycle (0-100 %)
  icr = 639; // icr and frecuency values are set to generate a 25 kHZ signal
  frecuency = 16000000 / (1 + icr);

  ICR1H = icr >> 8;
  ICR1L = icr & 0x00ff;
  // Sets the duty cycle with the value entered in line 12 
  OCR1A = icr * (duty_cycle / 100.0);
}

void loop() {
  //Prints the current duty cycle on the serial monitor every second

  duty_cycle = 10;
  OCR1A = icr * (duty_cycle / 100.0);
  Serial.print(" Duty cycle (%) = ");
  Serial.println(duty_cycle);
  delay(5000);

  duty_cycle = 90;
  OCR1A = icr * (duty_cycle / 100.0);
  Serial.print(" Duty cycle (%) = ");
  Serial.println(duty_cycle);
  delay(5000);

}