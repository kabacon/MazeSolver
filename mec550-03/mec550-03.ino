int motorPin = 12;

void setup() {
  pinMode(motorPin, OUTPUT);
}

void loop() {
  motorOnThenOff();
}

void motorOnThenOff() {
  int onTime = 5000;
  int offTime = 3000;
  digitalWrite(motorPin, HIGH);
  delay(onTime);
  digitalWrite(motorPin, LOW);
  delay(offTime);
}
