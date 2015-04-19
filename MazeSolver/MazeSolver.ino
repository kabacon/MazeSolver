#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>

#include <Graph.h>
#include <Node.h>

#include <PID_v1.h>      // This is the PID library from http://playground.arduino.cc/Code/PIDLibrary
#include <Encoder.h>     // This is the encoder library from http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Servo.h>       // Servo library


#ifndef DEBUG
//#define DEBUG         // Comment out this line to disable Serial output text (Uses memory and makes the program
#endif                  //                                                       run slower. Tuned parameters 
                        //                                                       will behave differently.)
#include "Constants.h"  // Constants and global variables are here
#include "Debug.h"      // Debug output stuff is here

/*
  Objects from the various libraries are created here.
*/
// PID controller object
PID pid(&irDistanceDiff, &turnAmount, &setPoint, kP, kI, kD, DIRECT);    // The PID control object

// Motor encoder objects
Encoder encoderLeft(pinEncoderL1, pinEncoderL2);    // These keep track of the encoder rotation
Encoder encoderRight(pinEncoderR1, pinEncoderR2);

// Servo object
Servo servo;


//Node* previousNode;


/*
  Program functions

  =================================================================================================================
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
  
  // Initialize the servo
  servo.attach(pinServo);
  servo.write(SERVO_UP);
  
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
    } else if (digitalRead(button3)) {
      servo.write(SERVO_UP);
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
    turnAroundState();
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
    
  case STATE_SCANNING:
    scanningState();
    break;
    
  case STATE_CENTERING_BALL:
    centerBallState();
    break;
    
  case STATE_MOVING_TO_BALL:
    moveToBallState();
    break;
    
  case STATE_BALL_CAPTURE:
    ballCaptureState();
    break;
    
  case STATE_BALL_CAPTURE_LEFT:
    ballCaptureLeft();
    break;
  
  case STATE_BALL_CAPTURE_RIGHT:
    ballCaptureRight();
    break;  
  
  case STATE_MOVE_BACK_FROM_BALL:
    moveBackFromBallState();
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
  State Behaviors defined here
  
  ==================================================================================================================
*/



/*
  Take IR sensor readings, and use the PID controller to drive between the walls on 
  the left and right. Check to see that the robot should still be moving forward
*/
void moveForwardState() {
  // Take sensor readings
  takeSensorReadings();
  ballCaptured = digitalRead(pinBallFdbk);
  
  // First, if the ball area is reached, change state
  if (!ballCaptured && lightReading <= RED_MAX && lightReading >= RED_MIN) { //  
    currentState = STATE_SCANNING;
    /*
    moveForward(32, 0);
    delay(400);
    */
    zeroEncoders();
    moveForward(0, scanningSpeed);
    return;
  }
  
  // If the walls are far away to the left or right, the robot shouldn't be moving
  // forward. Switch state.
  if (distanceLeft > maxWallDistance) {
    // There is no wall to the left
    if (distanceRight > maxWallDistance) {
      // There is no wall to the left AND right
      zeroEncoders();
      currentState = STATE_MOVE_UP_BLIND;   // Change the state
      return;                              // Don't continue this function
      
    } else {
      // Just no wall to the left
      zeroEncoders();
      currentState = STATE_MOVE_UP_FOLLOW_RIGHT;   // Change the state
      return;                                     // Don't continue this function
    }
  }
  
  if (distanceRight > maxWallDistance) {
    // Just no wall to the right
    zeroEncoders();
    currentState = STATE_MOVE_UP_FOLLOW_LEFT;   // Change the state
    return;                                    // Don't continue this function
  }
  
  // If there is a wall directly in front of the robot (and walls to the left and 
  // right), turn around 180 degrees.
  if (distanceFront1 < minWallDistance) {
    zeroEncoders();
    currentState = STATE_TURNING_AROUND;   // Change the state
    return;                               // Don't continue this function
  } 
  
  /*
    Now the robot should definitely be moving forward. Use the PID controller to 
    navigate down the hallway.
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
  takeSensorReadings();
  
  if ((encoderLeft.read() > encoderLZero + distanceToEncoderCount(moveUpDist) && 
            encoderRight.read() > encoderRZero + distanceToEncoderCount(moveUpDist)) || 
            distanceFront1 < minWallDistance) {
    zeroEncoders();
    
    // Add a node to the graph here <------------------------------------------------------------------------------
    
    // Decide which direction to turn
    if (distanceRight > maxWallDistance) {                            // Turn Right
      zeroEncoders();
      //stopAndResetEncoders();
      currentState = STATE_TURNING_RIGHT;
      return;
    } else if (distanceFront1 > maxWallDistance + minWallDistance) {  // Go Straight
      resetEncoders();                                                // Reset the encoders, since this is the start
      moveForward(fwdSpeed, 0);                                       // of a new path in the right direction
      currentState = STATE_MOVING_THROUGH_INTERSECTION;
      return;
    } else if (distanceLeft > maxWallDistance) {                      // Turn Left
      zeroEncoders();
      //stopAndResetEncoders();
      currentState = STATE_TURNING_LEFT;
      return;
    } else {                                                          // Turn Around
      zeroEncoders();
      //stopAndResetEncoders();
      currentState = STATE_TURNING_AROUND;
      return;
    }
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
  if ((encoderLeft.read() > distanceToEncoderCount(moveOutDist) + encoderLZero && 
            encoderRight.read() > distanceToEncoderCount(moveOutDist) + encoderRZero) || 
            distanceFront1 < minWallDistance) {
    zeroEncoders();
    //stopAndResetEncoders();
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
  if (encoderLeft.read() < rotationToEncoderCount(fullTurnRot) + encoderLZero && 
            encoderRight.read() > rotationToEncoderCount(fullTurnRot) + encoderRZero) {
    // The robot is done turning, move up out of the intersection
    resetEncoders();
    moveForward(fwdSpeed, 0);
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
  if (encoderLeft.read() > rotationToEncoderCount(fullTurnRot) + encoderLZero &&
          encoderRight.read() < rotationToEncoderCount(fullTurnRot) + encoderRZero) {
    // The robot is done turning, move up out of the intersection
    resetEncoders();
    moveForward(fwdSpeed, 0);
    currentState = STATE_MOVING_THROUGH_INTERSECTION;
  }
  
  // Otherwise, keep turning
  moveForward(0, -turnSpeed);
}

/*
  Turn the robot 180 degrees right
*/
void turnAroundState() {
  // Check the encoder counters to see if the robot is done turning
  if (encoderLeft.read() > 2 * rotationToEncoderCount(fullTurnRot) + encoderLZero && 
            encoderRight.read() < 2 * rotationToEncoderCount(fullTurnRot) + encoderRZero) {
    // The robot is done turning, move up out of the intersection
    resetEncoders();
    moveForward(fwdSpeed, 0);
    currentState = STATE_MOVING_THROUGH_INTERSECTION;
  }
  
  // Otherwise, keep turning
  moveForward(0, -turnSpeed);
}

/*
  Scan left and right to find the ball
*/
void scanningState() {
  //Move forward to the wall
  while (encoderLeft.read() < encoderLZero + distanceToEncoderCount(7.15) &&
        encoderRight.read() < encoderRZero + distanceToEncoderCount(7.15)) {
    moveForward(scanningSpeed, 0);
  }
  while (encoderLeft.read() > encoderLZero + distanceToEncoderCount(4.5) && 
        encoderRight.read() > encoderRZero + distanceToEncoderCount(4.5)) {
    moveForward(-scanningSpeed, 0);  
  }
  moveForward(0, 0);
  currentState = STATE_BALL_CAPTURE;
    
}

/*
  Center the ball in front of the light sensor
*/
void centerBallState() {
  takeSensorReadings();
  if (lastDistanceFront1 < distanceFront1) {
    zeroEncoders();
    currentState = STATE_MOVING_TO_BALL;
  }
}

/*
  After the ball is centered, move up to the ball
*/
void moveToBallState() {
  takeSensorReadings();
  if (distanceFront1 > 5.8) {
    moveForward(10,0);
  } else {
    moveForward(0,0);
    currentState = STATE_BALL_CAPTURE;
  }
}

/*
  Lower the gate to capture the ball. If the ball was captured, back away. If not,
  open the gate before backing away
*/
void ballCaptureState() {
    servo.write(SERVO_DOWN);
    delay(1500);
    ballCaptured = digitalRead(pinBallFdbk);
    if (!ballCaptured) {
      servo.write(SERVO_UP);
      zeroEncoders();
      currentState = STATE_BALL_CAPTURE_LEFT;
    } else {
      currentState = STATE_MOVE_BACK_FROM_BALL;
    }
}

void ballCaptureLeft() {
  if (encoderRight.read() < encoderRZero + rotationToEncoderCount(25)) {
    moveForward(0, scanningSpeed);
  } else {
    moveForward(scanningSpeed, 0);
    delay(250);
    moveForward(0,0);
    servo.write(SERVO_DOWN);
    delay(1500);
    moveForward(-scanningSpeed, 0);
    delay(250);
    ballCaptured = digitalRead(pinBallFdbk);
    if (!ballCaptured) {
      servo.write(SERVO_UP);
      currentState = STATE_BALL_CAPTURE_RIGHT;
    } else {
      moveForward(0, -scanningSpeed);
      while (encoderRight.read() > encoderRZero);
      moveForward(0, 0);
      currentState = STATE_MOVE_BACK_FROM_BALL;
    }
  }
}

void ballCaptureRight() {
  if (encoderLeft.read() < encoderLZero + rotationToEncoderCount(25)) {
    moveForward(0, -scanningSpeed);
  } else {
    moveForward(scanningSpeed, 0);
    delay(250);
    moveForward(0,0);
    servo.write(SERVO_DOWN);
    delay(1500);
    moveForward(-scanningSpeed, 0);
    delay(250);
    ballCaptured = digitalRead(pinBallFdbk);
    if (!ballCaptured) {
      servo.write(SERVO_UP);
      // Move back to center and move up a bit
      moveForward(0, scanningSpeed);
      while (encoderLeft.read() > encoderLZero);
      moveForward(0, 0);
      
      currentState = STATE_BALL_CAPTURE;
    } else {
      moveForward(0, scanningSpeed);
      while (encoderLeft.read() > encoderLZero);
      moveForward(0, 0);
      currentState = STATE_MOVE_BACK_FROM_BALL;
    }
  }
}

/*
  Back away from the ball location (whether the ball has been captured or not)
*/
void moveBackFromBallState() {
  if (encoderLeft.read() > encoderLZero && encoderRight.read() > encoderRZero) {
    moveForward(-10, 0);
  } else if (ballCaptured) {
    currentState = STATE_TURNING_AROUND;
    moveForward(0,0);
  } else {
    moveForward(0, -scanningSpeed);
    currentState = STATE_SCANNING;
  }
}



/*
  Some helper functions
  
  ===================================================================================================================
*/



/*
  Set the zero point on both encoders. Used when the robot needs to switch state, but 
  is still traveling on the same path.
*/
void zeroEncoders() {
  encoderLZero = encoderLeft.read();
  encoderRZero = encoderRight.read();
}

/*
  Reset both encoders to zero. Used when the robot begins to move down a new path.
*/
void resetEncoders() {
  encoderLeft.write(0);
  encoderRight.write(0);
  encoderLZero = 0;
  encoderRZero = 0;
}

/*
  Take IR sensor readings for each sensor, and convert the raw number to a distance 
*/
void takeSensorReadings() {
  
  irLeft = analogRead(pinIrL);
  irRight = analogRead(pinIrR);
  irFront1 = analogRead(pinIrF1);
//  irFront2 = analogRead(pinIrF2);    // No sensor here yet
  lightReading = analogRead(pinLightSensor);
  
  // Calibrate sensors if needed
  // irRight -= 30;
  
  // Convert the voltage readings to distances
  lastDistanceFront2 = lastDistanceFront1;
  lastDistanceFront1 = distanceFront1;
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
  forward by setting the left and right motor speeds. Positive turnAmount turns LEFT.
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
  Convert a distance in CM into an encoder count
*/
int distanceToEncoderCount(double cm) {
  return (int)(cm * 2000 / 17.0);
}

/*
  Convert a rotation angle in DEG into an encoder count
*/
int rotationToEncoderCount(double deg) {
  return (int)(deg * 1000 / 123.0);
}

/*
  Get an angle in DEG from the rotation of an encoder
*/
double encoderCountToAngle(int count) {
  return (double)count * 123.0 / 1000.0; 
}

