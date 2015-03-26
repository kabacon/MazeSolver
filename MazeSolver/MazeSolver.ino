#include <PID_v1.h>      // This is the PID library
#include <Encoder.h>     // This is the encoder library

#ifndef DEBUG
//#define DEBUG          // Comment out this line to disable Serial output text
#endif

/*
  All of the Arduino pin connections are assigned here.
*/
// Output pins
int pinMotorL = 5;          // Each motor uses three pins. A PWM signal is sent to the
int pinMotorR = 6;          // first pin, which sets the speed of the motor. The second
int pinMotorDirL1 = 4;      // and third pins define the direction of the motor. When 
int pinMotorDirL2 = 7;      // direction pin 1 is LOW and direction pin 2 is HIGH, the 
int pinMotorDirR1 = 8;      // motor will spin CCW. When direction pin 1 is HIGH and 
int pinMotorDirR2 = 9;      // direction pin 2 is LOW, the motor spins CCW
int pinServo = 10;          // I'm assuming we'll add a servo at some point

// Analog input pins
int pinIrL = A0;            // Left IR sensor
int pinIrR = A1;            // Right
int pinIrF1 = A2;           // Front 1          (Two front sensors for detecting the ping
int pinIrF2 = A3;           // Front 2           pong ball, unless we find another way) 

// Digital input pins
int button1 = 11;
int button2 = 12;

// Interrupt pins
int pinEncoderL1 = 2;      // The encoders use interrupt pins, which interrupt the
int pinEncoderL2 = 3;      // program in the middle of its loop whenever the encoder
int pinEncoderR1 = 0;      // switches state. This ensures each state switch is counted,
int pinEncoderR2 = 1;      // even if the motor is spinning faster than the loop runs.

// PID Controller
double kP = 10.0f;         // Proportional gain
double kI = 0.0f;          // Integral gain
double kD = 0.0f;          // Derivative gain
double irDistanceDiff;     // Difference between the left and right IR sensor distances
double turnAmount;         // Amount that the robot should turn
double setPoint = 0.0f;    // The ideal value for irSensorDiff
int fwdSpeed = 64;         // The forward speed, set to 64 for now until it can be tuned to run faster
PID pid(&irDistanceDiff, &turnAmount, &setPoint, kP, kI, kD, DIRECT);    // The PID control object

// IR Sensor readings
int irLeft = 0;            // Raw sensor readings
int irRight = 0;
int irFront1 = 0;
int irFront2 = 0;
double distanceLeft = 0;   // Interpolated distances from the sensor readings
double distanceRight = 0;
double distanceFront1 = 0;
double distanceFront2 = 0;

bool started = false;      // This becomes true once the program has started, allows the 
                           // robot to start with a button press
bool turnLeft = false;     // Is the robot turning left? This is used to allow the PID
                           // controller to turn in both directions

// Motor encoder objects
Encoder encoderLeft(pinEncoderL1, pinEncoderL2);    // These keep track of the encoder rotation
Encoder encoderRight(pinEncoderR1, pinEncoderR2);


/*
  PROGRAM STARTS HERE - Configure all digital input and output pins, 
  initialize the PID controller. If DEBUG is defined at the top, begin
  a serial connection for testing purposes.
*/
void setup() {
  // Set the output pins
  pinMode(pinMotorL, OUTPUT);
  pinMode(pinMotorR, OUTPUT);
  pinMode(pinMotorDirL1, OUTPUT);
  pinMode(pinMotorDirL2, OUTPUT);
  pinMode(pinMotorDirR1, OUTPUT);
  pinMode(pinMotorDirR2, OUTPUT);
  
  // Set the digital input pins
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  
  // Initialize the PID controller
  pid.SetMode(AUTOMATIC);
  
  // If DEBUG is defined at the top, start a serial connection
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
}


/*
  Once the setup function runs, the loop function runs continuously in a loop until power 
  is turned off or the reset button is pressed. Do all the really important stuff here.
*/
void loop() {
  // Wait for button press to start the robot
  while (!started) {
    if (digitalRead(button1) == HIGH) {
      started = true;
    } else {
      #ifdef DEBUG                      // If debugging, indicate that
      Serial.print("Not started: ");    // the program hasn't started
      Serial.println(millis());         // yet
      #endif
      delay(50);
    }
  }

  // Take sensor readings
  irLeft = analogRead(pinIrL);
  irRight = analogRead(pinIrR);
  irFront1 = analogRead(pinIrF1);
  
  // Calibrate sensors if needed
  // irRight -= 30;
  
  // Convert the voltage readings to distances
  distanceLeft = irDistanceFromVoltage(irLeft);      // irDistanceFromVoltage is a
  distanceRight = irDistanceFromVoltage(irRight);    // function, defined below.
  distanceFront1 = irDistanceFromVoltage(irFront1);
  
  // Compute the difference between the left and right sensors, and tell the PID
  // controller whether it should turn left or right to compensate.
  irDistanceDiff = (distanceRight - distanceLeft);
  if (irDistanceDiff > 0) {
    irDistanceDiff *= -1;
    turnLeft = false;
  } else {
    turnLeft = true;
  }
  
  // Tell the PID controller to calculate the turnAmount.
  pid.Compute();
  
  // Now that the turnAmount has been calculated, tell the motors to move forward
  // with the appropriate turnAmount (and whether to turn left or right)
  if (turnLeft) {
    moveForward(fwdSpeed, turnAmount);    // moveForward is a function, defined below
  } else {
    moveForward(fwdSpeed, -turnAmount);
  }  
  
  // If DEBUG is defined, a whole bunch of output text is printed to the Serial monitor
  #ifdef DEBUG
  Serial.print("Left sensor:  ");
  Serial.println(irLeft);
  Serial.print("Left sensor Distance:  ");
  Serial.println(distanceLeft);
  Serial.print("Right sensor: ");
  Serial.println(irRight);
  Serial.print("Right sensor Distance: ");
  Serial.println(distanceRight);
  Serial.print("Differential: ");
  Serial.println(irDistanceDiff);
  Serial.print("Turning Left? :");
  Serial.println(turnLeft?"Yes":"No");
  Serial.print("Turn Amount: ");
  Serial.println(turnAmount);
  Serial.println();
  Serial.print("Front sensor 1: ");
  Serial.println(irFront1);
  Serial.print("Front sensor 1 Distance: ");
  Serial.println(distanceFront1);
//  Serial.print("Front sensor 2: ");            // No sensor here yet
//  Serial.println(irFront2);
//  Serial.print("Front sensor 2 Distance: ");
//  Serial.println(distanceFront2);
  Serial.print("Left encoder:  ");
  Serial.println(encoderLeft.read());
  Serial.print("Right encoder: ");
  Serial.println(encoderRight.read());
  Serial.println("============================");
  #endif
  
  // If the second physical button is pressed, stop the robot and wait for restart
  if (digitalRead(button2)) {
    started = false;
    analogWrite(pinMotorL, 0);
    analogWrite(pinMotorR, 0);
  }
}


/*
  Return a distance in cm from an IR sensor voltage. The equation was fit from the 
  IR sensor data sheet. A power function seemed to be a really good fit for the 
  conversion. This uses d = 5.0294 * V ^ -1.214 with a distance range of 1.75 < d < 18
*/
double irDistanceFromVoltage(int binary) {
  double voltage = binary * 5.0 / 1023;
  if (voltage > 2.4) return 1.75;
  if (voltage < 0.35) return 18.0;
  double distance = 5.0294 * pow(voltage, -1.214);
  return distance;
}


/*
  This function takes two parameters, velocity and turnAmount, and moves the robot
  forward by setting the left and right motor speeds.
*/
void moveForward(int velocity, double turnAmount) {
  setLeftMotor(velocity - turnAmount);        // setLeftMotor is a function, defined below
  setRightMotor(velocity + turnAmount);       // setRightMotor is a function, defined below
}


/*
  Set the left motor output with a pwm signal. Ties the direction pins to the 
  direction of of the pwm parameter.
*/
void setLeftMotor(int pwm) {
  pwm *= -1;    // Since this motor is facing the opposite direction
  if (pwm > 0) {
    digitalWrite(pinMotorDirL1, HIGH);
    digitalWrite(pinMotorDirL2, LOW);
    analogWrite(pinMotorL, pwm);
  } else {
    digitalWrite(pinMotorDirL1, LOW);
    digitalWrite(pinMotorDirL2, HIGH);
    analogWrite(pinMotorL, -pwm);
  }
}


/*
  Set the right motor output with a pwm signal. Ties the direction pins to the 
  direction of of the pwm parameter.
*/
void setRightMotor(int pwm) {
  if (pwm > 0) {
    digitalWrite(pinMotorDirR1, HIGH);
    digitalWrite(pinMotorDirR2, LOW);
    analogWrite(pinMotorR, pwm);
  } else {
    digitalWrite(pinMotorDirR1, LOW);
    digitalWrite(pinMotorDirR2, HIGH);
    analogWrite(pinMotorR, -pwm);
  }
}
