/******************************************************************************
 *
 * Quaduino - Intel Ultimate Engineering Experience
 * Firmware revision: 20130522-2320
 *
 * BASIC "+" QUADCOPTER DESIGN
 * ===========================
 *
 * 1. Rigid frame with four motors equidistant from center at N/S/E/W positions.
 * 2. Electronics and power source (battery) mounted firmly in center of frame.
 * 3. Sensors built-in to determine orientation, heading, velocity, and position.
 * 4. Microcontroller software controls individual motor speed to hover/fly quad.
 *
 * ARDUINO MYQUAD IMPLEMENTATION
 * =============================
 *
 * Power-on initial setup:
 * 1. Initialize info/control link. (Serial/nRF24LU1+)
 * 2. Read stored calibrations settings. (EEPROM)
 * 3. Initialize motion sensors. (MPU-9150, AK8975)
 * 4. Initialize motor control. (Servo, PID)
 *
 * Main full-time run loop:
 * 1. Read and process commands from info/control link.
 * 2. Read orientation/motion/position data from sensors.
 * 3. If motors enabled, adjust motor speed:
 * a. Assign appropriate PID algorithm tuning.
 * b. Compute PID algorithm updates.
 * c. Set new motor speed values.
 * 4. Write status to info/control link if status changed.
 *
 *******************************************************************************/

// main Arduino library
//#include <Arduino.h>	//included in myQuadEEPROM.h

// EEPROM library
#include <EEPROM.h>	//included in myQuadEEPROM.cpp

// sensor library files
#include <Wire.h>		//included in I2CDev.h
#include "I2Cdev.h"
//#include "MPU9150Lib.h"	//included in myQuadSensors.h
//#include "dmpKey.h"	//included in inv_mpu_dmp_motion_driver.cpp
//#include "dmpmap.h"	//included in inv_mpu_dmp_motion_driver.cpp
//#include "inv_mpu.h"	//included in MPU9150Lib.cpp
//#include "inv_mpu_dmp_motion_driver.h"	//included in MPU9150Lib.cpp

// motor control library files
#include "PID_v1.h"

//#define DEBUG_MotorSketch
//#define DEBUG_DataSketch
//#define DEBUG_RadioDataSketch
#define DEBUG_FullFunctionSketch

// internal code organization for simplicity
#include "QuaduinoGlobal.h"
#include "QuaduinoEEPROM.h"	//included in MPU9150Lib.h
#include "QuaduinoSensors.h"
#include "QuaduinoFlight.h"
#include "QuaduinoCommunication.h"

void setupPins();

unsigned long previousTime = 0;

// =============================================================
// INITIAL SETUP
// =============================================================
void setup()
{
  Serial.write("Beginning setup");
  setupPins();

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  commDevice.init();
  sensorPackage.init();
  delay(1000);
  flightController.init();
  delay(1000);
  flightController.setupPidControl();
  Serial.println("Setup complete. Running flight loop in:");

  for(uint8_t t = 3; t > 0; t--) {
    Serial.print(t);
    Serial.println("... ");
    delay(1000);
  }
  Serial.println("0... ");
  delay(1000);
}

void setupPins() {
  pinMode(SYSOFF, INPUT);
  pinMode(VBAT, INPUT);

  pinMode(BAT_CHG, INPUT);
  pinMode(PGOOD, INPUT);

  pinMode(FRONT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(REAR_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);

  pinMode(ALT_INT, INPUT);
}

// =============================================================
// MAIN PROGRAM LOOP
// =============================================================

unsigned long frameCount;
uint32_t lastMic;
uint16_t benchMic = 13000;

void loop() {
#ifdef DEBUG_MotorSketch
  //Motor Sketch:
  analogWrite(FRONT_MOTOR, 0);
  analogWrite(RIGHT_MOTOR, 0);
  analogWrite(REAR_MOTOR, 0);
  analogWrite(LEFT_MOTOR, 0);
#elif defined DEBUG_DataSketch
  //Data Sketch:
  sensorPackage.updateMPU();
  sensorPackage.updateBarometer();
  sensorPackage.updateBattery();

  if(frameCount % 3 == 0) {
    commDevice.serialSendQuadStatus(sensorPackage, flightController, 0);
  }

  commDevice.parseSerialInput(0);

#elif defined DEBUG_RadioDataSketch
  //Radio Data Sketch:
  sensorPackage.updateMPU();
  sensorPackage.updateBarometer();
  sensorPackage.updateBattery();

  if (frameCount % 3 == 0) {
    commDevice.serialSendQuadStatus(sensorPackage, flightController, 1);
  }

  commDevice.parseSerialInput(1);

#elif defined DEBUG_FullFunctionSketch
  // Full-function sketch:
  // read and parse any available serial data
  //commDevice.parseSerialInput(1);

  sensorPackage.updateMPU();
  sensorPackage.updateBarometer();
  sensorPackage.updateBattery();
  /*
  if (frameCount % 3 == 0) {
    commDevice.serialSendQuadStatus(sensorPackage, flightController, 1);
  }
  */

  //if(flightController.areMotorsActive() && sensorPackage.getBatteryPercent()>= 60) {
    // Flight Commands
    flightController.hover(&frameCount, 0, 3);
    
    flightController.increaseAltitude(&frameCount, 3, 9, 30);
    
    flightController.hover(&frameCount, 9, 10);
    
    flightController.moveLeft(&frameCount, 10, 15, 25);
    flightController.moveForward(&frameCount, 15, 20, 25);
    flightController.moveRight(&frameCount, 20, 25, 25);
    flightController.moveBackward(&frameCount, 25, 30, 25);
    
    flightController.hover(&frameCount, 30, 40);
    
    flightController.killMotors(&frameCount, 18, 20);
    
    /*
    void QuaduinoFlight::increaseAltitude(unsigned long *currentFrame, unsigned long startFrame, unsigned long endFrame, uint8_t throttle) 
    
    
        
    if(frameCount <= 150) {
      Serial.println("Waiting.");
    } else if(frameCount >= 150 && frameCount < 300) {
      Serial.println("increasing altitude.");
      flightController.setThrottle(35);
    } else if(frameCount >= 300 && frameCount < 2000 ) {
      Serial.println("maintaining altitude.");
      flightController.hover();
    } else if(frameCount >= 2000 ) {
      analogWrite(FRONT_MOTOR, 0);
      analogWrite(REAR_MOTOR, 0);
      analogWrite(RIGHT_MOTOR, 0);
      analogWrite(LEFT_MOTOR, 0);
    } else {
      Serial.println();
    }
    */
  
    
    // Adjust all PIDs before applying to motors
    if(flightController.adjustRoll(sensorPackage.getRoll()) && flightController.adjustPitch(sensorPackage.getPitch())){
      
    }
  //}
  
  // Every 13500 microseconds, increment our frame counter.
  // 74.074074074.. frames per second
  if(micros() - lastMic > benchMic) {
    lastMic = micros();
    frameCount++;
  }
#endif 


  
}

