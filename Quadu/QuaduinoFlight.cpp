#include "QuaduinoFlight.h"
#include "QuaduinoGlobal.h"
#include <Arduino.h>

// =============================================================
// MOTOR CONTROL INITIAL SETUP
// =============================================================

QuaduinoFlight::QuaduinoFlight() {

}

void QuaduinoFlight::init() {
  Serial.print("... ");
  motorsActive = false;

  minPulseWidth = 0;
  maxPulseWidth = 255;
  gravityHover = 70; //proportional to voltage of battery... :(
  
  frontMotorOffset = 0;
  rearMotorOffset = 0;
  rightMotorOffset = 0;
  leftMotorOffset = 0;

  consKp = 0.3;
  consKi = 0.003;
  consKd = 0.192;

  aggKp = 0.93;
  aggKi = 0.1;
  aggKd = 0.2;

  angleToSwap = 15;

  analogWrite(FRONT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0);
  analogWrite(REAR_MOTOR, 0);
  analogWrite(LEFT_MOTOR, 0);

  Serial.println("motors complete.");
}

// =============================================================
// PID CONTROL INITIAL SETUP
// =============================================================

void QuaduinoFlight::setupPidControl() {
  // A1510 continuous-rotation motor notes (yr. 2012):
  //   write() effective range: [37,79]
  //   writeMilliseconds() effective range: [1100,1600]
  //   hover speed is ~1500 with myQuad
  //   PID output range is +/- 200, based on hover speed
  
  // Brushed Micro-motor (yr. 2013)
  //   effective range:[1, 255] - all of PWM range
  //   hover speed is ~55 with Quaduino
  //   Expect PID range to be +/- 255, to account for it being at full speed and needing a drastic change
  
  Serial.print("... ");
  
  float ultimateGain = 0.6;
  float oscillationPeriod = 1.25;
  yawPID.init(&yawInput, &yawOutput, &yawSetpoint, ultimateGain*0.6, 1.2*ultimateGain/oscillationPeriod, 0.6*ultimateGain*oscillationPeriod/8, DIRECT);
  ultimateGain = 0.6;
  oscillationPeriod = 1.25;
  pitchPID.init(&pitchInput, &pitchOutput, &pitchSetpoint, ultimateGain*0.6, 1.2*ultimateGain/oscillationPeriod, 0.6*ultimateGain*oscillationPeriod/8, DIRECT);
  ultimateGain = 0.6;
  oscillationPeriod = 1.25;
  rollPID.init(&rollInput, &rollOutput, &rollSetpoint, ultimateGain*0.6, 1.2*ultimateGain/oscillationPeriod, 0.6*ultimateGain*oscillationPeriod/8, DIRECT);
  
  pitchPID.SetOutputLimits(-55, 55);
  pitchPID.SetSampleTime(10); // 10
  pitchPID.SetMode(AUTOMATIC);
  pitchTuningActive = true;

  rollPID.SetOutputLimits(-55, 55);
  rollPID.SetSampleTime(10);
  rollPID.SetMode(AUTOMATIC);
  rollTuningActive = true;

  yawPID.SetOutputLimits(-55, 55);
  yawPID.SetSampleTime(10);
  yawPID.SetMode(AUTOMATIC);
  yawTuningActive = true;

  //altitude_tuning_active = false;

  yawSetpoint = 0.0; // no rotation
  rollSetpoint = 182; // level
  pitchSetpoint = 90; // level
  altitudeSetpoint = 0.0; // no change
  
  Serial.println("PIDs complete");
}

// =============================================================
// SET ALL MOTORS TO FULL THROTTLE
// =============================================================

void QuaduinoFlight::runMotorsMax() {
  // set all motors to maximum pulse width (highest speed)
  analogWrite(FRONT_MOTOR, maxPulseWidth);
  analogWrite(RIGHT_MOTOR, maxPulseWidth);
  analogWrite(REAR_MOTOR, maxPulseWidth);
  analogWrite(LEFT_MOTOR, maxPulseWidth);
}


// =============================================================
// SET ALL MOTORS TO ZERO THROTTLE
// =============================================================

void QuaduinoFlight::runMotorsMin() {
  // set all motors to minimum pulse width (lowest speed)
  analogWrite(FRONT_MOTOR, minPulseWidth);
  analogWrite(RIGHT_MOTOR, minPulseWidth);
  analogWrite(REAR_MOTOR, minPulseWidth);
  analogWrite(LEFT_MOTOR, minPulseWidth);
}


// =============================================================
// RUN MOTORS TO FULL THROTTLE, THEN ZERO THROTTLE
// =============================================================

void QuaduinoFlight::runMotorsFullrange() {
  runMotorsMax();
  delay(500);
  runMotorsMin();
  delay(500);
}

void QuaduinoFlight::runMotorsFullrangeGradual() {
  int range = minPulseWidth;
  bool increase = true;

  runMotorsMin();

  while(range <= maxPulseWidth && range >= minPulseWidth) {
    if(increase) {
      range++;
    }
    else if (increase && range == maxPulseWidth) {
      range--;
      increase = false;
    }
    else if(!increase) {
      range--;
    }
    else if(!increase && range == minPulseWidth) {
      runMotorsAt(0);
      break;
    }

    runMotorsAt(range);
  }
}

// =============================================================
// ADJUST PITCH USING PID/STEERING
// =============================================================

boolean QuaduinoFlight::adjustPitch(double pitchIn) { //Aiming for ~0.5
  if (pitchTuningActive) {
    // use orientation's "pitch" angle in degrees (passed as argument)
    pitchInput = pitchIn;
    //pitchSetpoint = 0.5; // should be = 0.5 +/- what we want to move by..

    /*
    // determine appropriate PID tunings
    double temp = pitchInput - pitchSetpoint;
    if (abs(temp) > angleToSwap) {
      // far away from target value, so use aggressive tunings
      (*pitchPID).SetTunings(aggKp, aggKi, aggKd);
    }
    else {
      // close to target value, so use conservative tunings
      (*pitchPID).SetTunings(consKp, consKi, consKd);
    }
    */

    // run PID computation
    if(pitchPID.Compute()) {
      
      rearMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + rearMotorOffset + yawAdjustFR + pitchOutput;
      frontMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + frontMotorOffset + yawAdjustFR - pitchOutput;
  
      analogWrite(REAR_MOTOR, rearMotorValue);
      analogWrite(FRONT_MOTOR, frontMotorValue);
      return true;
    }
  }
  
  // assign new motor speed settings
  
  return false;
}


// =============================================================
// ADJUST ROLL USING PID/STEERING
// =============================================================

boolean QuaduinoFlight::adjustRoll(double rollIn) { //looking for 90
  if (rollTuningActive) {
    // use orientation's "roll" angle in degrees (passed as argument)
    if(rollIn < 360/2) {
      rollInput = map(rollIn, 0, 180, 180, 0);
    } else if(rollIn >= 360/2) {
      rollInput = map(rollIn, 180, 360, 360, 180);
    }

    // determine appropriate PID tunings
    /*
    double temp = rollInput - rollSetpoint;
    if (abs(temp) > angleToSwap) {
      // far away from target value, so use aggressive tunings
      (*rollPID).SetTunings(aggKp, aggKi, aggKd);
    }
    else {
      // close to target value, so use conservative tunings
      (*rollPID).SetTunings(consKp, consKi, consKd);
    }
    */

    // run PID computation
    if(rollPID.Compute()) {
      
      leftMotorValue = minPulseWidth + gravityHover + altitudeSetpoint - rollOutput + leftMotorOffset + yawAdjustLR;
      rightMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + rollOutput + rightMotorOffset + yawAdjustLR;

      analogWrite(LEFT_MOTOR, leftMotorValue);
      analogWrite(RIGHT_MOTOR, rightMotorValue);
      return true;
    }
  }
  return false;
}


void QuaduinoFlight::increaseAltitude(unsigned long *currentFrame, unsigned long startSeconds, unsigned long timePeriod, uint8_t throttle) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    altitudeSetpoint = throttle;
    Serial.println("increasing alt.");
  }
}

void QuaduinoFlight::hover(unsigned long *currentFrame, unsigned long startSeconds, unsigned long timePeriod) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    altitudeSetpoint = 0.0;
    Serial.println("hovering.");
  }
}

void QuaduinoFlight::moveLeft(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endFrame, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    leftMotorOffset = -travelSpeed;
    rightMotorOffset = travelSpeed;
  } else {
    leftMotorOffset = 0;
    rightMotorOffset = 0;
  }
}

void QuaduinoFlight::moveRight(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endFrame, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    leftMotorOffset = travelSpeed;
    rightMotorOffset = -travelSpeed;
  } else {
    leftMotorOffset = 0;
    rightMotorOffset = 0;
  }
}

void QuaduinoFlight::moveForward(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endFrame, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    rearMotorOffset = travelSpeed;
    frontMotorOffset = -travelSpeed;
  } else {
    rearMotorOffset = 0;
    frontMotorOffset = 0;
  }
}

void QuaduinoFlight::moveBackward(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endFrame, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    rearMotorOffset = -travelSpeed;
    frontMotorOffset = travelSpeed;
  } else {
    rearMotorOffset = 0;
    frontMotorOffset = 0;
  }
}

void QuaduinoFlight::turnToHeading(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endFrame, uint16_t targetHeading) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    
  }
}

void QuaduinoFlight::killMotors(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endFrame) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = startFrame + timePeriod*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame) {
    runMotorsMin();
    rollTuningActive = false;
    pitchTuningActive = false;
  }
}


// =============================================================
// ADJUST YAW USING PID/STEERING (NOT IMPLEMENTED YET)
// =============================================================

boolean QuaduinoFlight::adjustYaw(double yawIn) { //looking to hold a heading.. let's try 0 degrees
  if (yawTuningActive) {
    // use orientation's "yaw" angle in degrees (passed as argument)
    yawInput = sensorPackage.getYaw();
    //yawSetpoint = 0; //we really don't have a set heading. Need to create an average based on where we're pointing.
    yawPID.Compute();
  }
}


void QuaduinoFlight::setRecvPitch(uint16_t recvPitch) {
  rcvPitch = recvPitch;
}

void QuaduinoFlight::setRecvRoll(uint16_t recvRoll) {
  rcvRoll = recvRoll;
}

void QuaduinoFlight::setRecvYaw(uint16_t recvYaw) {
  rcvYaw = recvYaw;
}

void QuaduinoFlight::setRecvThrottle(uint16_t recvThrottle) {
  rcvThrottle = recvThrottle;
}

void QuaduinoFlight::setRecvActive(uint16_t recvActive) {
  if(recvActive == 'a') {
    motorsActive = true;
  }
}

void QuaduinoFlight::incMotorOffset(char motor, float inc) {
  switch(motor) {
  case 'f':
    frontMotorOffset += inc;
    break;
  case 'r':
    rightMotorOffset += inc;
    break;
  case 'b':
    rearMotorOffset += inc;
    break;
  case 'l':
    leftMotorOffset += inc;
    break;
  }
}

void QuaduinoFlight::setRollTuningActive(bool act) {
  rollTuningActive = act;
}

void QuaduinoFlight::setPitchTuningActive(bool act) {
  pitchTuningActive = act;
}

void QuaduinoFlight::setYawTuningActive(bool act) {
  yawTuningActive = act;
}


void QuaduinoFlight::incTuningParam(char param, float inc) {
  switch(param) {
  case 'y':
    consKp += inc;
    break;
  case 'u':
    consKi += inc;
    break;
  case 'i':
    consKd += inc;
    break;
  case 'h':
    consKp -= inc;
    break;
  case 'j':
    consKi -= inc;
    break;
  case 'k':
    consKd -= inc;
    break;
  case 'o':
    aggKp += inc;
    break;
  case 'p':
    aggKi += inc;
    break;
  case '[':
    aggKd += inc;
    break;
  case 'l':
    aggKp -= inc;
    break;
  case ';':
    aggKi -= inc;
    break;
  case '\'':
    aggKd -= inc;
    break;
  default:
    break;
  }
}

bool QuaduinoFlight::getRollTuningActive() {
  return rollTuningActive;
}

bool QuaduinoFlight::getPitchTuningActive() {
  return pitchTuningActive;
}

bool QuaduinoFlight::getYawTuningActive() {
  return yawTuningActive;
}

uint8_t QuaduinoFlight::getFrontMotorValue() {
  return frontMotorValue;
}

uint8_t QuaduinoFlight::getRightMotorValue() {
  return rightMotorValue;
}

uint8_t QuaduinoFlight::getRearMotorValue() {
  return rearMotorValue;
}

uint8_t QuaduinoFlight::getLeftMotorValue() {
  return leftMotorValue;
}

boolean QuaduinoFlight::areMotorsActive() {
  //if(digitalRead(SYS_OFF)) { //if usb is plugged in..
  //  return false;
  //} 
  return true;
}
