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
const int pinServo = 10;          // Servo pin

// Analog input pins
const int pinIrL = A0;            // Left IR sensor
const int pinIrR = A1;            // Right
const int pinIrF1 = A2;           // Front 1          (Two front sensors for detecting the ping
const int pinIrF2 = A4;           // Front 2           pong ball, unless we find another way) 
const int pinLightSensor = A3;    // Light sensor

// Digital input pins
const int button1 = 11;           // Forward button
const int button2 = 12;           // Back button
const int button3 = A5;           // Middle button
const int pinBallFdbk = 13;       // Ball capture feedback button

// Interrupt pins
const int pinEncoderL1 = 3;      // The encoders use interrupt pins, which interrupt the
const int pinEncoderL2 = 2;      // program in the middle of its loop whenever the encoder
const int pinEncoderR1 = 1;      // switches state. This ensures each state switch is counted,
const int pinEncoderR2 = 0;      // even if the motor is spinning faster than the loop runs.

/*
  Constants are declared here
*/

// Movement parameters
const int fwdSpeed = 48;         // The forward speed, set to 64 for now until it can be tuned to run faster
const int turnSpeed = 48;        // Turning speed for corners
const int moveUpSpeed = 48;
const int checkDistance = 2.0;   // Distance to check moving up into an intersection
const int moveUpDist = 6.8 - checkDistance;     // Encoder count for moving up into an intersection
const int moveOutDist = 5.5;    // Encoder count for moving out of an intersection
const int fullTurnRot = 90;   // Encoder count for making a 90 degree turn


const double maxWallDistance = 6.0f;  // Define the maximum wall distance in cm. If a wall is 
                                      // farther than, the robot will respond as if there is
                                      // no wall there. (This should be tuned)
const double minWallDistance = 5.8f;  // Define the minimum wall distance in cm (for the front).
                                      // If a wall is closer than this, the robot will stop, since
                                      // there's a wall right in front of it. (This should be tuned)

// Light sensor constants
const int GREEN_MIN = 300;  // 300
const int GREEN_MAX = 350;  // 350
const int RED_MIN = 380;    // 380
const int RED_MAX = 420;    //420
const int WHITE_MIN = 430;  // 430

// Servo constants
const int SERVO_DOWN = 180;
const int SERVO_UP = 0;

// Ball capture parameters
const int scanningSpeed = 16;
const double ballThreshold = 1.0;

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
const int STATE_MOVING_THROUGH_BLIND =        23;
const int STATE_MOVING_THROUGH_FOLLOW_LEFT =  24;
const int STATE_MOVING_THROUGH_FOLLOW_RIGHT = 25;
const int STATE_PUSH_BALL =                   31;
const int STATE_CENTERING_BALL =              32;
const int STATE_MOVING_TO_BALL =              33;
const int STATE_BALL_CAPTURE =                34;
const int STATE_BALL_CAPTURE_LEFT =           35;
const int STATE_BALL_CAPTURE_RIGHT =          36;
const int STATE_MOVE_BACK_FROM_BALL =         37;

const int NORTH = 0;
const int EAST =  1;
const int SOUTH = 2;
const int WEST =  3;

