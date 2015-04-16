#include <Servo.h>

Servo servo;
int pos;
int servoPin = 3;

void setup() {
  servo.attach(servoPin);
}

void loop() {
  servo.write(90);
  delay(1000);
  servo.write(135);
  delay(1000);
  servo.write(45);
  delay(1000);
}

