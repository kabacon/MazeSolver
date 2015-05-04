#ifndef DEBUG
//#define DEBUG         // Comment out this line to disable Serial output text (Uses ~600 bytes of memory 
//extern void outputDebugText();
#endif                  // and makes the program run slower.

#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>

#include <Graph.h>
#include <Node.h>

#include <PID_v1.h>      // This is the PID library from http://playground.arduino.cc/Code/PIDLibrary
#include <Encoder.h>     // This is the encoder library from http://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Servo.h>       // Servo library

#ifdef DEBUG
#include <MemoryFree.h>
#endif

#include "Constants.h"  // Constants and global variables are here
//#include "Debug.h"      // Debug output stuff is here

#define RIGHT_WALL_FOLLOWING    // Define one of these two to set the robot to left or right wall following  
//#define LEFT_WALL_FOLLOWING   // during the mapping phase.

/*
  Global variables are declared here.
*/
// PID Controller
double kP = 30.0f;         // Proportional gain
double kI = 0.0f;          // Integral gain
double kD = 0.0f;          // Derivative gain
double irDistanceDiff;     // Difference between the left and right IR sensor distances
double turnAmount;         // Amount that the robot should turn
double setPoint = 0.0f;    // The ideal value for irSensorDiff
double targetDistance = 2.0f;

// IR Sensor readings
int irLeft = 0;            // Raw sensor readings
int irRight = 0;
int irFront1 = 0;
int irFront2 = 0;
double distanceLeft = 0;   // Interpolated distances from the sensor readings
double distanceRight = 0;
double distanceFront1 = 0;
double distanceFront2 = 0;
double lastDistanceFront1 = 0;  // For comparing the change in front distance
double lastDistanceFront2 = 0;

// Light sensor reading
int lightReading = 0;

// Moving zeroes for the encoders
int encoderLZero = 0;
int encoderRZero = 0;

bool started = false;      // This becomes true once the program has started, allows the 
                           // robot to start with a button press
bool turnLeft = false;     // Is the robot turning left? This is used to allow the PID
                           // controller to turn in both directions 

bool ballCaptured = false; // Does the robot have the ball?
int captureAttempts = 0;
int numRedSamples = 0;

// Movement Direction
int currentDirection = NORTH;  // Robot always starts north

// Whether or not the map is complete, and the robot is following a path
bool mapComplete = false;
bool followingPath = false;
int finishDirection = 0;

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

// Graph for mapping the maze,  initial node:(north, south, east, west)
Graph graph(true, false, false, false);
            
                         
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
  pid.SetSampleTime(20);
  pid.SetOutputLimits(0, 28);
 
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
    pausedState();
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
    moveUpFollowLeftState();    // Just move up blindly for now
    break;
  
  // The robot is moving up to an intersection, following the wall to its right.
  // Make sure there is still a wall to follow, and check to see what's in front.
  // Check the encoders to see if it's done moving up.  
  case STATE_MOVE_UP_FOLLOW_RIGHT:
    moveUpFollowRightState();    // Just move up blindly for now
    break;
  
  // The robot is moving up to an intersection without any walls to follow (a T
  // or a four-way intersection). Check the encoders to see if it's done moving up   
  case STATE_MOVE_UP_BLIND:
    moveUpBlindState();
    break;
  
  // The robot has gone into an intersection, turned, and must now move out of the
  // intersection.  
  case STATE_MOVING_THROUGH_BLIND:
    moveUpThroughBlindState();
    break; 
    
  case STATE_MOVING_THROUGH_FOLLOW_RIGHT:
    moveUpThroughFollowRightState();
    break; 
    
  case STATE_MOVING_THROUGH_FOLLOW_LEFT:
    moveUpThroughFollowLeftState();
    break;
    
  case STATE_PUSH_BALL:
    pushingBallState();
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
  Check to see what buttons have been pressed and respond to the input. If the mapping is complete 
  and the start button is pressed, calculate a path from start to finish and follow it.
*/
void pausedState() {
    // If the start button is pushed, start the robot
    if (digitalRead(button1) == HIGH) {
      // If there is a map of the maze, calculate a path from start to finish and follow it
      if (mapComplete) {
        graph.calculateShortestPath(graph.getStart(), graph.getFinish());
        followingPath = true;
        ballCaptured = false;
        captureAttempts = 0;
        currentDirection = graph.advancePath();  // Advance the path and assign the current direction
      }
      
      currentState = STATE_MOVING_FORWARD;
      delay(100);  // Short delay
      
    } else if (digitalRead(button3)) {
      // If the middle button was pressed, release the ball.
      servo.write(SERVO_UP);
      
    } else {
      // Otherwise, just wait.
      delay(50);
      #ifdef DEBUG                      // If debugging, indicate that
      Serial.print("Not started: ");    // the program hasn't started
      Serial.println(millis());         // yet
      #endif
    }
}

/*
  Take IR sensor readings, and use the PID controller to drive between the walls on 
  the left and right. Check to see that the robot should still be moving forward
*/
void moveForwardState() {
  // Take sensor readings
  takeSensorReadings();

  // First, if the ball area is reached, change state  
  if (!ballCaptured && lightReading <= RED_MAX && lightReading >= RED_MIN && numRedSamples > 20) {
    // If mapping, don't add a node for the finish area. Instead, just note the direction.
    if (!mapComplete) {
      graph.foundFinish();
      finishDirection = currentDirection;
      mapComplete = true;
    }
    
    currentState = STATE_PUSH_BALL;
    
    zeroEncoders();
    moveForward(0, scanningSpeed);
    return;
  }
  
  /*
    Reached the start
  */
  /*if (ballCaptured && followingPath && lightReading < GREEN_MAX) {
    moveForward(0, 0);
    currentState = STATE_PAUSED;
  }*/
  
  
  // There's an opening to the left or right, check to see that it's not just a crack
  if (distanceLeft > maxWallDistance || distanceRight > maxWallDistance) {
    moveForward(fwdSpeed, 0);
    zeroEncoders();
    while (encoderRight.read() < encoderRZero + distanceToEncoderCount(checkDistance));
    takeSensorReadings();
    /*
    if (followingPath && (distanceLeft > maxWallDistance || distanceRight > maxWallDistance)) {
      zeroEncoders();
      currentState = STATE_MOVE_UP_BLIND;
      return;
    }*/
      
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
        currentState = STATE_MOVE_UP_FOLLOW_RIGHT;   // Move up follow right
        targetDistance = distanceRight;
        if (targetDistance < 2.0) targetDistance = 2.0;
        return;                                     // Don't continue this function
      }
    }
    
    if (distanceRight > maxWallDistance) {
      // Just no wall to the right
      zeroEncoders();
      currentState = STATE_MOVE_UP_FOLLOW_LEFT;   // Change the state
      targetDistance = distanceLeft;
      if (targetDistance < 2.0) targetDistance = 2.0;
      return;                                    // Don't continue this function
    }
  }
  // If there is a wall directly in front of the robot (and walls to the left and 
  // right), turn around 180 degrees.
  if (distanceFront1 < minWallDistance && !mapComplete) {
    int L = (encoderLeft.read() + encoderRight.read()) / 2;    // Add a dead-end node to the graph
    graph.addNode(false, false, false, currentDirection, abs(L));
    currentDirection = (currentDirection + 2) % 4;
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
  
  // Done turning
  if ((encoderLeft.read() > encoderLZero + distanceToEncoderCount(moveUpDist) && 
            encoderRight.read() > encoderRZero + distanceToEncoderCount(moveUpDist)) || 
            distanceFront1 < minWallDistance) {
    zeroEncoders();
              
    // If still mapping the maze, add a node to the graph here
    if (!mapComplete) {
      int L = (encoderLeft.read() + encoderRight.read()) / 2;    // Add a node to the graph
      bool forward = distanceFront1 > maxWallDistance;
      bool left = distanceLeft > maxWallDistance;
      bool right = distanceRight > maxWallDistance;
      graph.addNode(forward, left, right, currentDirection, abs(L));
    }
    
    // If the map has been completed, but a path hasn'
    
    // Decide which direction to turn. If the robot is following a path, that decision comes 
    // from that path. Otherwise, do right wall following.
    if (followingPath) {
      // Advance the path by 1 and decide which direction to turn
      int nextDirection = graph.advancePath(); 
      
      // Reached the end of the path.
      if (nextDirection == -1) { 
        // If the ball hasn't been captured, the robot has reached the last node before the
        // finish area. Turn in the direction of the finish area and move up.
        if (!ballCaptured) {
          followingPath = false;
          if (finishDirection == currentDirection) {              // Go straight
            moveForward(fwdSpeed, 0);
            currentState = STATE_MOVING_THROUGH_BLIND;
            return;
          } else if (finishDirection == (currentDirection + 1) % 4) {  // Turn right
            currentDirection = finishDirection;
            currentState = STATE_TURNING_RIGHT;
            return;
          } else if (finishDirection == (currentDirection + 2) % 4) {  // Turn around
            currentDirection = finishDirection;
            currentState = STATE_TURNING_AROUND;
            return;
          } else if (finishDirection == (currentDirection + 3) % 4) {  // Turn left
            currentDirection = finishDirection;
            currentState = STATE_TURNING_LEFT;
            return;
          } else {
            // Something went wrong ... 
            moveForward(0, 0);
            currentState = STATE_PAUSED;
            return;
          }
          
        } else {
          // Otherwise, the robot reached the start node.
          followingPath = false;
          moveForward(0, 0);
          currentState = STATE_PAUSED;
          return;
        }
        
      } else if (nextDirection == currentDirection) {            // Go straight
        currentState = STATE_MOVING_THROUGH_BLIND;
        return;
      } else if (nextDirection == (currentDirection + 1) % 4) {  // Turn right
        currentDirection = nextDirection;
        currentState = STATE_TURNING_RIGHT;
        return;
      } else if (nextDirection == (currentDirection + 2) % 4) {  // Turn around
        currentDirection = nextDirection;
        currentState = STATE_TURNING_AROUND;
        return;
      } else if (nextDirection == (currentDirection + 3) % 4) {  // Turn left
        currentDirection = nextDirection;
        currentState = STATE_TURNING_LEFT;
        return;
      } else {
        // Something went wrong ... 
        currentState = STATE_PAUSED;
      }
      
    // Not following a path, do left or right wall following    
    } else {
      #ifdef RIGHT_WALL_FOLLOWING
      if (distanceRight > maxWallDistance ) {//&& numRightTurns < 4) {                            // Turn Right
        currentDirection = (currentDirection + 1) % 4;
        currentState = STATE_TURNING_RIGHT;
        return;
      } else if (distanceFront1 > maxWallDistance + minWallDistance) {  // Go Straight
        moveForward(moveUpSpeed, 0);
        resetEncoders();
        if (distanceRight < maxWallDistance) {
          currentState = STATE_MOVING_THROUGH_FOLLOW_RIGHT;
        } else if (distanceLeft < maxWallDistance) {
          currentState = STATE_MOVING_THROUGH_FOLLOW_LEFT;
        } else {
          currentState = STATE_MOVING_THROUGH_BLIND;
        }
        return;
      } else if (distanceLeft > maxWallDistance) {                      // Turn Left
        currentDirection = (currentDirection + 3) % 4;
        currentState = STATE_TURNING_LEFT;
        return;
      } else {                                                          // Turn Around
        currentDirection = (currentDirection + 2) % 4;
        currentState = STATE_TURNING_AROUND;
        return;
      }
      #endif/*
      #ifdef LEFT_WALL_FOLLOWING
      if (distanceLeft > maxWallDistance) {                             // Turn Left
        currentDirection = (currentDirection + 3) % 4;
        currentState = STATE_TURNING_LEFT;
        return;
      } else if (distanceFront1 > maxWallDistance + minWallDistance) {  // Go Straight
        moveForward(moveUpSpeed, 0);
        resetEncoders();
        currentState = STATE_MOVING_THROUGH_BLIND;
        return;
      } else if (distanceRight > maxWallDistance) {                     // Turn Right
        currentDirection = (currentDirection + 1) % 4;
        currentState = STATE_TURNING_RIGHT;
        return;
      } else {                                                          // Turn Around
        currentDirection = (currentDirection + 2) % 4;
        currentState = STATE_TURNING_AROUND;
        return;
      }
      #endif*/
    }
    
  } 
  
  // If the robot isn't fully into the intersection, continue to move up blindly
  moveForward(moveUpSpeed, 0);
}

void moveUpFollowLeftState() {
  takeSensorReadings();
  if ((encoderLeft.read() > encoderLZero + distanceToEncoderCount(moveUpDist) && 
            encoderRight.read() > encoderRZero + distanceToEncoderCount(moveUpDist)) || 
            distanceFront1 < minWallDistance) {
    currentState = STATE_MOVE_UP_BLIND;
  }
  // Compute the difference between the left and right sensors, and tell the PID
  // controller whether it should turn left or right to compensate.
  irDistanceDiff = distanceLeft - targetDistance;
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
    moveForward(moveUpSpeed, -turnAmount);    // moveForward is a function, defined below
  } else {
    moveForward(moveUpSpeed, turnAmount);
  }
}


void moveUpFollowRightState() {
  if ((encoderLeft.read() > encoderLZero + distanceToEncoderCount(moveUpDist) && 
            encoderRight.read() > encoderRZero + distanceToEncoderCount(moveUpDist)) || 
            distanceFront1 < minWallDistance) {
    currentState = STATE_MOVE_UP_BLIND;
  }
  // Compute the difference between the left and right sensors, and tell the PID
  // controller whether it should turn left or right to compensate.
  takeSensorReadings();
  irDistanceDiff = distanceRight - targetDistance;
  if (irDistanceDiff > 0) {
    irDistanceDiff *= -1;
    turnLeft = false;
  } else {
    turnLeft = true;
  }
  
  // Tell the PID controller to calculate the turnAmount.
  //fpid.SetTunings(kP / 2.0, kI / 2.0, 0);
  pid.Compute();
  
  // Now that the turnAmount has been calculated, tell the motors to move forward
  // with the appropriate turnAmount (and whether to turn left or right)
  if (turnLeft) {
    moveForward(moveUpSpeed, turnAmount);    // moveForward is a function, defined below
  } else {
    moveForward(moveUpSpeed, -turnAmount);
  }
}



/*
  Move the robot up out of an intersection without following a wall to the left or right
*/
void moveUpThroughBlindState() {
  // Check the encoder counters to see if the robot is done moving up, or check to see if
  // there's a wall directly in front for some reason.
  if ((encoderLeft.read() > distanceToEncoderCount(moveOutDist) + encoderLZero && 
            encoderRight.read() > distanceToEncoderCount(moveOutDist) + encoderRZero) || 
            distanceFront1 < minWallDistance) {
    currentState = STATE_MOVING_FORWARD;
  } 
  //
  // Check to see if there's something to the right and left, so the robot can resume navigation
  /*takeSensorReadings();
  if (distanceLeft < maxWallDistance && distanceRight < maxWallDistance) {
    currentState = STATE_MOVING_FORWARD;
  }
  */
  // If the robot isn't fully out of the intersection, continue to move up blindly
  moveForward(moveUpSpeed, 0);

}

void moveUpThroughFollowLeftState() {
  if ((encoderLeft.read() > distanceToEncoderCount(moveOutDist) + encoderLZero && 
            encoderRight.read() > distanceToEncoderCount(moveOutDist) + encoderRZero) || 
            distanceFront1 < minWallDistance) {
    currentState = STATE_MOVING_FORWARD;
  }
  // Compute the difference between the left and right sensors, and tell the PID
  // controller whether it should turn left or right to compensate.
  irDistanceDiff = distanceLeft - targetDistance;
  if (irDistanceDiff > 0) {
    irDistanceDiff *= -1;
    turnLeft = false;
  } else {
    turnLeft = true;
  }
  
  // Tell the PID controller to calculate the turnAmount.
  //pid.SetTunings(kP /2.0, kI / 2.0, 0);
  pid.Compute();
  
  // Now that the turnAmount has been calculated, tell the motors to move forward
  // with the appropriate turnAmount (and whether to turn left or right)
  if (turnLeft) {
    moveForward(moveUpSpeed, -turnAmount);    // moveForward is a function, defined below
  } else {
    moveForward(moveUpSpeed, turnAmount);
  }      
}

void moveUpThroughFollowRightState() {
  if ((encoderLeft.read() > distanceToEncoderCount(moveOutDist) + encoderLZero && 
            encoderRight.read() > distanceToEncoderCount(moveOutDist) + encoderRZero) || 
            distanceFront1 < minWallDistance) {
    currentState = STATE_MOVING_FORWARD;
  } 
  
  // Compute the difference between the left and right sensors, and tell the PID
  // controller whether it should turn left or right to compensate.
  takeSensorReadings();
  irDistanceDiff = distanceRight - targetDistance;
  if (irDistanceDiff > 0) {
    irDistanceDiff *= -1;
    turnLeft = false;
  } else {
    turnLeft = true;
  }
  
  // Tell the PID controller to calculate the turnAmount.
  //fpid.SetTunings(kP / 2.0, kI / 2.0, 0);
  pid.Compute();
  
  // Now that the turnAmount has been calculated, tell the motors to move forward
  // with the appropriate turnAmount (and whether to turn left or right)
  if (turnLeft) {
    moveForward(moveUpSpeed, turnAmount);    // moveForward is a function, defined below
  } else {
    moveForward(moveUpSpeed, -turnAmount);
  }
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
    moveForward(moveUpSpeed, 0);
    takeSensorReadings();
    if (distanceLeft < maxWallDistance) {
      targetDistance = distanceLeft;
      if (targetDistance < 2.0) targetDistance = 2.0;
      currentState = STATE_MOVING_THROUGH_FOLLOW_LEFT;
    } else {
      currentState = STATE_MOVING_THROUGH_BLIND;
    }
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
    moveForward(moveUpSpeed, 0);
    takeSensorReadings();
    if (distanceRight < maxWallDistance) {
      targetDistance = distanceRight;
      if (targetDistance < 2.0) targetDistance = 2.0;
      currentState = STATE_MOVING_THROUGH_FOLLOW_RIGHT;
    } else {
      currentState = STATE_MOVING_THROUGH_BLIND;
    }
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
    
    resetEncoders();
    
    // If the ball was captured, but a path hasn't been calculated yet, find one!
    if (ballCaptured && !followingPath) {
      // (It shouldn't actually take very long to calculate the path, but stop to be safe)
      moveForward(0, 0);
      // Calculate the shortest path back from finish to start
      delay(500);
      graph.calculateShortestPath(graph.getFinish(), graph.getStart());
      followingPath = true;
      currentDirection = (finishDirection + 2) % 4;
      currentState = STATE_MOVING_FORWARD;
      moveForward(fwdSpeed, 0);
      return;
      
    // If the robot got back to the start with the ball, stop.
    } else if (ballCaptured && lightReading < GREEN_MAX) {
      currentDirection = NORTH;
      currentState = STATE_PAUSED;
      followingPath = false;
      moveForward(0, 0);
    } else {
      // The robot is done turning, move up out of the intersection
      moveForward(fwdSpeed, 0);
      currentState = STATE_MOVING_THROUGH_BLIND;
    }
    
  }
  
  // Otherwise, keep turning
  moveForward(0, -turnSpeed);
}

/*
  Scan left and right to find the ball
*/
void pushingBallState() {
  //Move forward to the wall
  while (encoderLeft.read() < encoderLZero + distanceToEncoderCount(6.0) &&
        encoderRight.read() < encoderRZero + distanceToEncoderCount(6.0)) {
    moveForward(scanningSpeed, 0);
    
  }
  zeroEncoders();
  while (encoderLeft.read() > encoderLZero + distanceToEncoderCount(-3.0) && 
        encoderRight.read() > encoderRZero + distanceToEncoderCount(-3.0)) {
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
    moveForward(0, 0);
    // Align for 1 second
    align();
    moveForward(0, 0);
    servo.write(SERVO_DOWN);
    delay(1500);
    ballCaptured = digitalRead(pinBallFdbk);
    captureAttempts++;
    if (!ballCaptured) {
      servo.write(SERVO_UP);
      zeroEncoders();
      currentState = STATE_BALL_CAPTURE_LEFT;
    } else {
      currentDirection = (currentDirection + 2) % 4;
      currentState = STATE_MOVE_BACK_FROM_BALL;
    }
}

void ballCaptureLeft() {
  if (encoderRight.read() < encoderRZero + rotationToEncoderCount(20)) {
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
      currentDirection = (currentDirection + 2) % 4;
      moveForward(0, -scanningSpeed);
      while (encoderRight.read() > encoderRZero);
      moveForward(0, 0);
      currentState = STATE_MOVE_BACK_FROM_BALL;
    }
  }
}

void ballCaptureRight() {
  if (encoderLeft.read() < encoderLZero + rotationToEncoderCount(20)) {
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
      if (captureAttempts < 2) {
        moveForward(scanningSpeed, 0);
        delay(500);
        moveForward(0, 0);
        currentState = STATE_BALL_CAPTURE;
      } else {
        moveForward(0, 0);
        // Align to the maze
        align();
        // Turn around and leave
        currentState = STATE_TURNING_AROUND;
        ballCaptured = true;
        currentDirection = (currentDirection + 2) % 4;
      }
    } else {
      moveForward(0, scanningSpeed);
      while (encoderLeft.read() > encoderLZero);
      moveForward(0, 0);
      currentDirection = (currentDirection + 2) % 4;
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
    // Align to the maze
    align();
    zeroEncoders();
    currentState = STATE_TURNING_AROUND;
    moveForward(0,0);
  } else {
    moveForward(0, -scanningSpeed);
    currentState = STATE_PUSH_BALL;
  }
}


void align() {
  unsigned long tStart = millis();
  while (millis() < tStart + 1000) {
    takeSensorReadings();
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
      moveForward(0, turnAmount);    // moveForward is a function, defined below
    } else {
      moveForward(0, -turnAmount);
    }
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
  // Read from the IR sensors and light sensor
  irLeft = analogRead(pinIrL);
  irRight = analogRead(pinIrR);
  irFront1 = analogRead(pinIrF1);
  lightReading = analogRead(pinLightSensor);
  if (lightReading > RED_MIN && lightReading < RED_MAX) numRedSamples++;
  else numRedSamples = 0;
  
  // Convert the voltage readings to distances
  lastDistanceFront2 = lastDistanceFront1;
  lastDistanceFront1 = distanceFront1;
  distanceLeft = irDistanceFromVoltage(irLeft);      // irDistanceFromVoltage is a
  distanceRight = irDistanceFromVoltage(irRight);    // function, defined below.
  distanceFront1 = irDistanceFromVoltage(irFront1);\
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

#ifdef DEBUG
#include "Debug.h"      // Debug output stuff is here

int getEncoderL() { return encoderLeft.read();}
int getEncoderR() { return encoderRight.read();}
#endif
