int pins[] = {13, 12, 11, 10, 9, 8, 7, 6};

int delayTime = 80;

void setup() {
  for (int i = 0; i < 8; ++i) {
    pinMode(pins[i], OUTPUT);
  }
  digitalWrite(pins[0], HIGH);
  digitalWrite(pins[1], LOW);
  delay(delayTime);
}

void loop() {
  for (int i = 1; i < 8; ++i) {
    digitalWrite(pins[i - 1], LOW);
    digitalWrite(pins[i], HIGH);
    delay(delayTime);
  }
  
  for (int i = 6; i >= 0; --i) {
    digitalWrite(pins[i + 1], LOW);
    digitalWrite(pins[i], HIGH);
    delay(delayTime);
  }
}
