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

  frontMotorSteering = 0;
  rearMotorSteering = 0;
  rightMotorSteering = 0;
  leftMotorSteering = 0;

  allowAutonomy = false;
  
  altitudeCurrent = 0;

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
  //   hover speed is ~70 with Quaduino
  //   Expect PID range to be +/- 55, to account for it being at full speed and needing a drastic change
  
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

  yawPID.SetOutputLimits(-10, 10);
  yawPID.SetSampleTime(10);
  yawPID.SetMode(AUTOMATIC);
  yawTuningActive = true;

  yawSetpoint = 0.0; // no rotation
  rollSetpoint = 90; // level
  pitchSetpoint = 180; // level
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

void QuaduinoFlight::runMotorsFullRange() {
  uint8_t value = 0;
  int ourDirection = 1;
  
  if(millis() % 1000 == 0) {
    value += ourDirection;
  }
  
  if(value >= 255) {
    ourDirection = -1;
  } else if(value <= 0) {
    ourDirection = 1;
  }
  
  analogWrite(FRONT_MOTOR, value);
  analogWrite(REAR_MOTOR, value);
  analogWrite(RIGHT_MOTOR, value);
  analogWrite(LEFT_MOTOR, value);
}

// =============================================================
// ADJUST PITCH USING PID/STEERING
// =============================================================

boolean QuaduinoFlight::adjustPitch(double pitchIn) { //Aiming for ~0.5
  if(pitchIn < 360/2) {
    pitchInput = map(pitchIn, 0, 180, 180, 0);
  } else if(pitchIn > 360/2) {
    pitchInput = map(pitchIn, 360, 180, 180, 360);
  }
  
  //Serial.println(pitchInput);
  
  if (pitchTuningActive && motorsActive) {
    // run PID computation
    if(pitchPID.Compute()) {
      rearMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + rearMotorOffset + yawAdjustFR + pitchOutput + rearMotorSteering;
      frontMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + frontMotorOffset + yawAdjustFR - pitchOutput + frontMotorSteering;
      
      analogWrite(REAR_MOTOR, rearMotorValue);
      analogWrite(FRONT_MOTOR, frontMotorValue);
      
      return true;
    }
  }
  return false;
}

// =============================================================
// ADJUST ROLL USING PID/STEERING
// =============================================================

boolean QuaduinoFlight::adjustRoll(double rollIn) { //looking for 90
  rollInput = rollIn;
  
  //Serial.println(rollInput);
  
  if (rollTuningActive && motorsActive) {
    // run PID computation
    if(rollPID.Compute()) {
      leftMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + rollOutput + leftMotorOffset + yawAdjustLR + leftMotorSteering;
      rightMotorValue = minPulseWidth + gravityHover + altitudeSetpoint - rollOutput + rightMotorOffset + yawAdjustLR + rightMotorSteering;
      
      analogWrite(LEFT_MOTOR, leftMotorValue);
      analogWrite(RIGHT_MOTOR, rightMotorValue);
      
      return true;
    }
  }
  return false;
}

boolean QuaduinoFlight::adjustYaw(double yawIn) { //looking to hold a heading.. let's try 0 degrees
  if (yawTuningActive && motorsActive) {
    // use orientation's "yaw" angle in degrees (passed as argument)
    yawInput = sensorPackage.getYaw();
    //yawSetpoint = 0; //we really don't have a set heading. Need to create an average based on where we're pointing.
    yawPID.Compute();
  }
}

void QuaduinoFlight::startAutonomy() {
  allowAutonomy = true;
  motorsActive = true;
}

void QuaduinoFlight::increaseAltitude(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t throttle) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    altitudeSetpoint = throttle;
    Serial.println("increasing alt.");
  }
}

void QuaduinoFlight::decreaseAltitude(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t invThrottle) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    altitudeSetpoint = -invThrottle;
    Serial.println("decreasing alt.");
  }
}

void QuaduinoFlight::hover(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    altitudeSetpoint = 0.0;
    Serial.println("hovering.");
  }
}

void QuaduinoFlight::moveLeft(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    leftMotorSteering = -travelSpeed;
    rightMotorSteering = travelSpeed;
    Serial.println("moving left.");
  } else {
    leftMotorSteering = 0;
    rightMotorSteering = 0;
  }
}

void QuaduinoFlight::moveRight(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    leftMotorSteering = travelSpeed;
    rightMotorSteering = -travelSpeed;
    Serial.println("moving right.");
  } else {
    leftMotorSteering = 0;
    rightMotorSteering = 0;
  }
}

void QuaduinoFlight::moveForward(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    rearMotorSteering = travelSpeed;
    frontMotorSteering = -travelSpeed;
    Serial.println("moving forward.");
  } else {
    rearMotorSteering = 0;
    frontMotorSteering = 0;
  }
}

void QuaduinoFlight::moveBackward(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    rearMotorSteering = -travelSpeed;
    frontMotorSteering = travelSpeed;
    Serial.println("moving backward.");
  } else {
    rearMotorSteering = 0;
    frontMotorSteering = 0;
  }
}

void QuaduinoFlight::turnToHeading(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint16_t targetHeading) {
  unsigned long startFrame = startSeconds*74;
  unsigned long endFrame = endSeconds*74;
  if(*currentFrame >= startFrame && *currentFrame < endFrame && allowAutonomy) {
    Serial.print("turning to heading: ");
    Serial.println(targetHeading);
  }
}

void QuaduinoFlight::killMotors(unsigned long *currentFrame, unsigned long startSeconds) {
  unsigned long startFrame = startSeconds*74;
  if(*currentFrame >= startFrame && allowAutonomy) {
    runMotorsMin();
    allowAutonomy = false;
    motorsActive = false;
    Serial.println("Killing motors. I hope you're on the ground.");
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
  } else {
    motorsActive = false;
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
