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

bool running = false;

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
  running = true;
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
  flightController.runMotorsFullRange();

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
   
   // Serial.println(sensorPackage.getBatteryPercent());
   //flightController.adjustPitch(sensorPackage.getPitch());
   if (flightController.checkOrientation(sensorPackage.getPitch(), sensorPackage.getRoll())){
     running = false;
   }

 
//   Serial.println(sensorPackage.getPitch());
  if(sensorPackage.getBatteryPercent() >= 60 && running) {
    // Flight Commands
    flightController.startAutonomy();
    flightController.increaseAltitude(&frameCount, 0, 3, 20);
    //flightController.increaseAltitude(&frameCount, 0, 1, 20);
    //flightController.moveRight(&frameCount, 3, 5, 7);
    //flightController.moveForward(&frameCount, 3, 8, 7);
   // flightController.moveForward(&frameCount, 8, 11, 10);
    //flightController.increaseAltitude(&frameCount, 3, 5, 10);
    //flightController.increaseAltitude(&frameCount, 4, 5, 20);
    flightController.hover(&frameCount, 3, 10);
    //flightController.moveLeft(&frameCount, 9, 11, 7);
    flightController.decreaseAltitude(&frameCount, 10, 15, 15);
    flightController.killMotors(&frameCount, 15);
    
    // Adjust all PIDs before applying to motors
    flightController.adjustRoll(sensorPackage.getRoll());
    flightController.adjustPitch(sensorPackage.getPitch());
   } else {
     flightController.killMotors(&frameCount, 0);
   }

  // Every 13500 microseconds, increment our frame counter.
  // 74.074074074.. frames per second
  if(micros() - lastMic > benchMic) {
    lastMic = micros();
    frameCount++;
  }
#endif


}


