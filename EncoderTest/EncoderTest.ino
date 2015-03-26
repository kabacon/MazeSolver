#include <Encoder.h>

Encoder encoderLeft(0, 1);
Encoder encoderRight(2, 3);

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Left Encoder:  ");
  Serial.println(encoderLeft.read());
  Serial.print("Right Encoder: ");
  Serial.println(encoderRight.read());
  Serial.println("====================");
  delay(50);
}

