#include <PID_v1.h>      // This is the PID library from http://playground.arduino.cc/Code/PIDLibrary
#include <Encoder.h>     // This is the encoder library from http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Servo.h>

#ifndef DEBUG
//#define DEBUG          // Comment out this line to disable Serial output text (Uses memory and makes the program
#endif                  //                                                       run slower. Tuned parameters 
                        //                                                       will behave differently.)
/*
  All of the Arduino pin connections are assigned here.
*/
// Output pins
const int pinMotorL = 5;          // Each motor uses three pins. A PWM signal is sent to the
const int pinMotorR = 6;          // first pin, which sets the speed of the motor. The second
const int pinMotorDirL1 = 4;      // and third pins define the direction of the motor. When 
const int pinMotorDirL2 = 7;      // direction pin 1 is LOW and direction pin 2 is HIGH, the 
const int pinMotorDirR1 = 8;      // motor will spin CCW. When direction pin 1 is HIGH and 
const int pinMotorDirR2 = 9;      // direction pin 2 is LOW, the motor spins CCW
const int pinServo = 10;          // I'm assuming we'll add a servo at some point

// Analog input pins
const int pinIrL = A0;            // Left IR sensor
const int pinIrR = A1;            // Right
const int pinIrF1 = A2;           // Front 1          (Two front sensors for detecting the ping
const int pinIrF2 = A3;           // Front 2           pong ball, unless we find another way) 

// Digital input pins
const int button1 = 11;
const int button2 = 12;

// Interrupt pins
const int pinEncoderL1 = 2;      // The encoders use interrupt pins, which interrupt the
const int pinEncoderL2 = 3;      // program in the middle of its loop whenever the encoder
const int pinEncoderR1 = 0;      // switches state. This ensures each state switch is counted,
const int pinEncoderR2 = 1;      // even if the motor is spinning faster than the loop runs.

/*
  Constants and global variables are declared here.
*/
// PID Controller
double kP = 10.0f;         // Proportional gain
double kI = 0.0f;          // Integral gain
double kD = 0.0f;          // Derivative gain
double irDistanceDiff;     // Difference between the left and right IR sensor distances
double turnAmount;         // Amount that the robot should turn
double setPoint = 0.0f;    // The ideal value for irSensorDiff


int fwdSpeed = 64;         // The forward speed, set to 64 for now until it can be tuned to run faster
int turnSpeed = 64;        // Turning speed for corners
int moveUpCount = 675;     // Encoder count for moving up into an intersection
int moveOutCount = 300;    // Encoder count for moving out of an intersection
int fullTurnCount = 710;   // Encoder count for making a 90 degree turn

// IR Sensor readings
int irLeft = 0;            // Raw sensor readings
int irRight = 0;
int irFront1 = 0;
int irFront2 = 0;
double distanceLeft = 0;   // Interpolated distances from the sensor readings
double distanceRight = 0;
double distanceFront1 = 0;
double distanceFront2 = 0;

const double maxWallDistance = 6.0f;  // Define the maximum wall distance in cm. If a wall is 
                                      // farther than, the robot will respond as if there is
                                      // no wall there. (This should be tuned)
const double minWallDistance = 2.0f;  // Define the minimum wall distance in cm (for the front).
                                      // If a wall is closer than this, the robot will stop, since
                                      // there's a wall right in front of it. (This should be tuned)

bool started = false;      // This becomes true once the program has started, allows the 
                           // robot to start with a button press
bool turnLeft = false;     // Is the robot turning left? This is used to allow the PID
                           // controller to turn in both directions

/*
  Objects from the various libraries are created here.
*/
// PID controller object
PID pid(&irDistanceDiff, &turnAmount, &setPoint, kP, kI, kD, DIRECT);    // The PID control object

// Motor encoder objects
Encoder encoderLeft(pinEncoderL1, pinEncoderL2);    // These keep track of the encoder rotation
Encoder encoderRight(pinEncoderR1, pinEncoderR2);

// Define different behavior states
int currentState;
const int STATE_PAUSED =                       0;    // The numbers here don't matter, it only matters 
const int STATE_MOVING_FORWARD =               1;    // that each one is unique. A behavior for each of
const int STATE_TURNING_LEFT =                10;    // these states needs to be defined.
const int STATE_TURNING_RIGHT =               11;
const int STATE_TURNING_AROUND =              12;
const int STATE_MOVE_UP_FOLLOW_LEFT =         20;   // When moving up to a corner, if there's a wall to the 
const int STATE_MOVE_UP_FOLLOW_RIGHT =        21;   // left or right, follow it. Otherwise, just move up 
const int STATE_MOVE_UP_BLIND =               22;   // blindly
const int STATE_MOVING_THROUGH_INTERSECTION = 23;

/*
  ==================================================================================================
*/

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
  
  // Start with the robot paused
  currentState = STATE_PAUSED;
  
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
  // All of the robot's behavior depends on what state it's in
  switch (currentState) {
    
  // The robot is paused, wait for a button press to start the robot
  case STATE_PAUSED:
    if (digitalRead(button1) == HIGH) {
      currentState = STATE_MOVING_FORWARD;
    } else {
      delay(50);
      #ifdef DEBUG                      // If debugging, indicate that
      Serial.print("Not started: ");    // the program hasn't started
      Serial.println(millis());         // yet
      #endif
    }
    break;
  
  // The robot is moving forward. Use the PID controller to follow the walls.  
  case STATE_MOVING_FORWARD:
    moveForwardState();  // Always move forward for now, but CHANGE THIS!!
    break;
  
  // The robot is turning left. Check the encoders to see if it's done turning.
  case STATE_TURNING_LEFT:
    turnLeftState();
    break;
  
  // The robot is turning right. Check the encoders to see if it's done turning.  
  case STATE_TURNING_RIGHT:
    turnRightState();
    break;
  
  //  For now, just move forward
  case STATE_TURNING_AROUND:
    currentState = STATE_MOVING_FORWARD;
    break;
  
  // The robot is moving up to an intersection, following the wall to its left.
  // Make sure there is still a wall to follow, and check to see what's in front.  
  // Check the encoders to see if it's done moving up.
  case STATE_MOVE_UP_FOLLOW_LEFT:
    currentState = STATE_MOVE_UP_BLIND;    // Just move up blindly for now
    break;
  
  // The robot is moving up to an intersection, following the wall to its right.
  // Make sure there is still a wall to follow, and check to see what's in front.
  // Check the encoders to see if it's done moving up.  
  case STATE_MOVE_UP_FOLLOW_RIGHT:
    currentState = STATE_MOVE_UP_BLIND;    // Just move up blindly for now
    break;
  
  // The robot is moving up to an intersection without any walls to follow (a T
  // or a four-way intersection). Check the encoders to see if it's done moving up   
  case STATE_MOVE_UP_BLIND:
    moveUpBlindState();
    break;
  
  // The robot has gone into an intersection, turned, and must now move out of the
  // intersection.  
  case STATE_MOVING_THROUGH_INTERSECTION:
    moveUpThroughIntersectionState();
    break;
  }

  // If DEBUG is defined, a whole bunch of output text is printed to the Serial monitor
  #ifdef DEBUG
  outputDebugText();
  #endif  
  
  // If the second physical button is pressed, stop the robot and wait for restart
  if (digitalRead(button2)) {
    currentState = STATE_PAUSED;
    analogWrite(pinMotorL, 0);
    analogWrite(pinMotorR, 0);
  }
}

/*
  Take IR sensor readings, and use the PID controller to drive between the walls on 
  the left and right. Check to see that the robot should still be moving forward
*/
void moveForwardState() {
  // Take IR sensor readings
  takeIrSensorReadings();
  
  // If the walls are far away to the left or right, the robot shouldn't be moving
  // forward. Switch state.
  if (distanceLeft > maxWallDistance) {
    // There is no wall to the left
    if (distanceRight > maxWallDistance) {
      // There is no wall to the left AND right
      stopAndResetEncoders();
      currentState = STATE_MOVE_UP_BLIND;   // Change the state
      return;                              // Don't continue this function
      
    } else {
      // Just no wall to the left
      stopAndResetEncoders();
      currentState = STATE_MOVE_UP_FOLLOW_RIGHT;   // Change the state
      return;                                     // Don't continue this function
    }
  }
  
  if (distanceRight > maxWallDistance) {
    // Just no wall to the right
    stopAndResetEncoders();
    currentState = STATE_MOVE_UP_FOLLOW_LEFT;   // Change the state
    return;                                    // Don't continue this function
  }
  
  // If there is a wall directly in front of the robot (and walls to the left and 
  // right), turn around 180 degrees.
  if (distanceFront1 < minWallDistance) {
    stopAndResetEncoders();
    currentState = STATE_TURNING_AROUND;   // Change the state
    return;                               // Don't continue this function
  } 
  
  /*
    Now the robot should definitely be moving forward.
  */
  
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
}

/*
  Move the robot up into an intersection without following a wall to the left or right
*/
void moveUpBlindState() {
  // Check the encoder counters to see if the robot is done moving up, or check to see if
  // there's a wall directly in front for some reason.
  if ((encoderLeft.read() > moveUpCount && encoderRight.read() > moveUpCount) || distanceFront1 < minWallDistance) {
    stopAndResetEncoders();
    
    // Decide which direction to turn
    if (distanceRight > maxWallDistance) {
      stopAndResetEncoders();
      currentState = STATE_TURNING_RIGHT;
      return;
    } else if (distanceLeft > maxWallDistance) {
      stopAndResetEncoders();
      currentState = STATE_TURNING_LEFT;
      return;
    } else {
      stopAndResetEncoders();
      currentState = STATE_TURNING_AROUND;
      return;
    }
    currentState = STATE_TURNING_LEFT;   // Change the state
    return;                                    // Don't continue this function
  } 
  
  // Continue to move up blindly into the intersection
  moveForward(fwdSpeed, 0);

}

/*
  Move the robot up out of an intersection without following a wall to the left or right
*/
void moveUpThroughIntersectionState() {
  // Check the encoder counters to see if the robot is done moving up, or check to see if
  // there's a wall directly in front for some reason.
  if ((encoderLeft.read() > moveOutCount && encoderRight.read() > moveOutCount) || distanceFront1 < minWallDistance) {
    stopAndResetEncoders();
    currentState = STATE_MOVING_FORWARD;
  } 
  
  // Continue to move up blindly through the intersection
  moveForward(fwdSpeed, 0);

}

/*
  Turn the robot 90 degrees left
*/
void turnLeftState() {
  // Check the encoder counters to see if the robot is done turning
  if (encoderLeft.read() < fullTurnCount && encoderRight.read() > fullTurnCount) {
    // The robot is done turning, move up out of the intersection
    stopAndResetEncoders();
    currentState = STATE_MOVING_THROUGH_INTERSECTION;
  }
  
  // Otherwise, keep turning
  moveForward(0, turnSpeed);
}

/*
  Turn the robot 90 degrees right
*/
void turnRightState() {
  // Check the encoder counters to see if the robot is done turning
  if (encoderLeft.read() > fullTurnCount && encoderRight.read() < fullTurnCount) {
    // The robot is done turning, move up out of the intersection
    stopAndResetEncoders();
    currentState = STATE_MOVING_THROUGH_INTERSECTION;
  }
  
  // Otherwise, keep turning
  moveForward(0, -turnSpeed);
}

/*
  Stop the robot and reset both encoder counters
*/
void stopAndResetEncoders() {
  moveForward(0, 0);                         // Stop the robot
  encoderLeft.write(0);                      // Reset the encoder counters
  encoderRight.write(0);
}

/*
  Take IR sensor readings for each sensor, and convert the raw number to a distance 
*/
void takeIrSensorReadings() {
  
  irLeft = analogRead(pinIrL);
  irRight = analogRead(pinIrR);
  irFront1 = analogRead(pinIrF1);
//  irFront2 = analogRead(pinIrF2);    // No sensor here yet
  
  // Calibrate sensors if needed
  // irRight -= 30;
  
  // Convert the voltage readings to distances
  distanceLeft = irDistanceFromVoltage(irLeft);      // irDistanceFromVoltage is a
  distanceRight = irDistanceFromVoltage(irRight);    // function, defined below.
  distanceFront1 = irDistanceFromVoltage(irFront1);
//  distanceFront2 = irDistanceFromVoltage(irFront2);
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


/*
  If DEBUG is defined, output debugging text to the serial monitor
*/
#ifdef DEBUG
void outputDebugText() {
  Serial.print("CURRENT STATE: ");
  Serial.println(stateToString(currentState));
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
  delay(100);
}

/*
  Convert a state variable to a String for a text description. Only used in debugging
*/
String stateToString(int state) {
  switch (state) {
  case STATE_PAUSED:
    return "Paused";
  case STATE_MOVING_FORWARD:
    return "Moving Forward";
  case STATE_MOVING_THROUGH_INTERSECTION:
    return "Moving Through Intersection";
  case STATE_TURNING_LEFT:
    return "Turning Left";
  case STATE_TURNING_RIGHT:
    return "Turning Right";
  case STATE_TURNING_AROUND:
    return "Turning Around";
  case STATE_MOVE_UP_FOLLOW_LEFT:
    return "Moving Up -- Following Left Wall";
  case STATE_MOVE_UP_FOLLOW_RIGHT:
    return "Moving Up -- Following Right Wall";
  case STATE_MOVE_UP_BLIND:
    return "Moving Up -- Blind";
  default:
    return "?????";
  }
}
#endif
