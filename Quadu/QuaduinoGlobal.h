#ifndef _myQuadPins_
#define _myQuadPins_

#include "QuaduinoSensors.h"
#include "QuaduinoCommunication.h"
#include "QuaduinoFlight.h"

// Pin Definitions
#define SYSOFF		49
#define VBAT		A8

#define BAT_CHG		32
#define PGOOD           53

#define FRONT_MOTOR	10	// Motor 1 output
#define RIGHT_MOTOR     8	// Motor 2 output
#define REAR_MOTOR	12	// Motor 3 output
#define LEFT_MOTOR	11	// Motor 4 output

#define ALT_INT		19

/* The following pins are defined by their respective classes. In general, you don't
 * need to define their pins. They are only placed here as a representation of
 * which pins are unavailable.
 *
#define SDA		20
#define SCL		21
// SDA/SDL are I2C pins and defined in I2Cdev
#define MISO		50
#define MOSI		51
#define SCK		52
// MOSI/MISO/SCK are SPI pins and are defined in SPI.h
#define Radio_TX	16
#define Radio_RX	17
// Radio_TX/Radio_RX are Serial
 *
 */

static QuaduinoSensors sensorPackage;
static QuaduinoCommunication commDevice;
static QuaduinoFlight flightController;

#endif
