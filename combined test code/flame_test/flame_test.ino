
int analogPin = A3; // KY-026 analog interface
int analogVal; //analog readings

void setup(){

  Serial.begin(9600);
}

void loop()
{

  // Read the analog interface
  analogVal = analogRead(analogPin); 
  Serial.println(analogVal); // print analog value to serial

  delay(100);
}