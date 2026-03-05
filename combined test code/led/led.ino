#define RED 6
#define GREEN 10
#define BLUE 11

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
}

void setColor(int r, int g, int b) {
  analogWrite(RED, r);
  analogWrite(GREEN, g);
  analogWrite(BLUE, b);
}

void loop() {

  // 红色
  setColor(255, 0, 0);
  delay(2000);

  // 绿色
  setColor(0, 255, 0);
  delay(2000);

  // 蓝色
  setColor(0, 0, 255);
  delay(2000);

  // 紫色
  setColor(255, 0, 255);
  delay(2000);
}
