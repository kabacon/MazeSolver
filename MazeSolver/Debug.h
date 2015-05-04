#ifdef DEBUG

/*
  Debug output text and helper functions. These will only be compiled if DEBUG is defined.
  
  =======================================================================================================================
*/



String stateToString(int state);
extern int getEncoderL();
extern int getEncoderR();

/*
  If DEBUG is defined, output debugging text to the serial monitor
*/
void outputDebugText() {/*
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
  Serial.print("Light sensor: ");
  Serial.println(lightReading);
  Serial.print("Left encoder:  ");
  Serial.println(getEncoderL());
  Serial.print("Right encoder: ");
  Serial.println(getEncoderR());
  Serial.println();*/
  Serial.print("Free memory: ");
  Serial.println(freeMemory());
  Serial.println("~600 bytes used for debug");
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
  case STATE_SCANNING:
    return "Scanning";
  case STATE_CENTERING_BALL:
    return "Centering Ball";
  case STATE_MOVING_TO_BALL:
    return "Moving to Ball";
  case STATE_BALL_CAPTURE:
    return "Ball Capture";
  case STATE_MOVE_BACK_FROM_BALL:
    return "Moving Back from Ball";
  default:
    return "?????";
  }
}
#endif
