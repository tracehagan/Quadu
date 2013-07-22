#ifndef _QuaduinoFlight_
#define _QuaduinoFlight_

#include <Arduino.h>
#include "PID_v1.h"

class QuaduinoFlight {
private:
  unsigned long altitudeCurrent;

  int frontMotorValue;
  int rightMotorValue;
  int rearMotorValue;
  int leftMotorValue;

  int frontMotorSteering;
  int rightMotorSteering;
  int rearMotorSteering;
  int leftMotorSteering;
  
  // minimum and maximum effective pulse widths
  // (continuous-rotation motors use PWM signals to control their speed)
  int minPulseWidth; // minimum pulse width
  int maxPulseWidth; // maximum pulse width

  // controls whether motors are enabled or not (useful to shut off quickly)
  boolean motorsActive; // = false;
  boolean allowAutonomy;
  
  // best initial guess at what "hover" motor speed should be,
  float gravityHover;


  // fine-tuning motor offset values to accomodate fixed imbalances
  // or differences in each motor, which there will be some
  float frontMotorOffset;
  float rearMotorOffset;
  float rightMotorOffset;
  float leftMotorOffset;

  // yaw adjustment offsets, for maintaining or changing heading
  // front/rear motors rotate in opposite direction from left/right
  // motors, so by increasion lift power on one set and decreasing
  // lift power on the other, you can get a gyroscopic-like rotation
  // effect without otherwise changing flight (hopefully)
  int16_t yawAdjustFR;
  int16_t yawAdjustLR;

  // yaw PID variables (Setpoint of 0 means due north)
  bool yawTuningActive;     // sets yaw control active or not
  double yawSetpoint;        // sets desired yaw angle (heading)
  double yawInput;           // will contain orientation's "yaw" angle in degrees
  double yawOutput;          // will control all motors to change/keep rotation

  // pitch PID variables (Setpoint of 0 means level, no pitch in either direction)
  bool pitchTuningActive;   // sets pitch control active or not
  double pitchSetpoint;      // sets desired pitch angle in degrees
  double pitchInput;         // will contain orientation's "pitch" angle in degrees
  double pitchOutput;        // will control front/rear motors

  // roll PID variables (Setpoint of 0 means level, no roll in either direction)
  bool rollTuningActive;    // sets roll control active or not
  double rollSetpoint;       // set desired roll angle in degrees
  double rollInput;          // will contain orientation's "roll" angle in degrees
  double rollOutput;         // will control left/right motors

  // altitude PID variables (Setpoint of 0 means no altitude change)
  bool altitudeActive;       // sets altitude control active or not
  double altitudeSetpoint;   // set desired altitude change
  double altitudeInput;      // will contain estimated vertical velocity
  double altitugeOutput;     // will control all motors to change/keep elevation
  // NOTE: ALTITUDE CONTROL IS NOT CURRENTLY IMPLEMENTED

  // adjustments sent by the control center for throttle/steering/etc.
  int16_t rcvYaw;
  int16_t rcvPitch;
  int16_t rcvRoll;
  int16_t rcvThrottle;
  uint8_t rcvActive;

  // Specify the links and initial tuning parameters
  PID yawPID;
  PID pitchPID;
  PID rollPID;
public:
  QuaduinoFlight();

  // =============================================================
  // MOTOR CONTROL INITIAL SETUP
  // =============================================================
  void init();

  // =============================================================
  // PID CONTROL INITIAL SETUP
  // =============================================================
  void setupPidControl();

  // =============================================================
  // RUN MOTORS IN VARIOUS PATTERNS
  // =============================================================
  void runMotorsMax();
  void runMotorsMin();
  void runMotorsFullRange();
  void runMotorsAt(int value);
  void runMotorsWithPID();
  
  // =============================================================
  // ADJUST PITCH/ROLL/YAW USING PID/STEERING
  // =============================================================
  boolean checkOrientation(double pitchIn, double rollIn);
  boolean adjustPitch(double pitchIn);
  boolean adjustRoll(double rollIn);
  boolean adjustYaw(double yawIn);
  
  // =============================================================
  // AUTONOMOUS FLIGHT CONTROLLERS
  // =============================================================
  void startAutonomy();
  void increaseAltitude(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t throttle);
  void decreaseAltitude(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t invThrottle);
  void hover(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds);
  void moveLeft(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed);
  void moveRight(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed);
  void moveForward(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed);
  void moveBackward(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint8_t travelSpeed);
  void turnToHeading(unsigned long *currentFrame, unsigned long startSeconds, unsigned long endSeconds, uint16_t targetHeading);
  void killMotors(unsigned long *currentFrame, unsigned long startSeconds);
  
  // Setters
  void setRecvPitch(uint16_t recvPitch);
  void setRecvRoll(uint16_t recvRoll);
  void setRecvYaw(uint16_t recvYaw);
  void setRecvThrottle(uint16_t recvThrottle);
  void setRecvActive(uint16_t recvActive);

  void incMotorOffset(char motor, float inc);

  void setRollTuningActive(bool act);
  void setPitchTuningActive(bool act);
  void setYawTuningActive(bool act);

  void incTuningParam(char param, float inc);

  //getters
  bool getRollTuningActive();
  bool getPitchTuningActive();
  bool getYawTuningActive();
 
  double getYawPIDOutput();
  double getRollPIDOutput();
  double getPitchPIDOutput();

  uint8_t getFrontMotorValue();
  uint8_t getRightMotorValue();
  uint8_t getRearMotorValue();
  uint8_t getLeftMotorValue();
  
  boolean areMotorsActive();
};
#endif

/*
#ifndef Flight_h
 #define Flight_h

 #include "Arduino.h"
 #include <Serial.h>
 #include <PID_v1.h>

 // The Motor class contains data and operations (functions) for one motor.
 // In the Motor class: "static Motor motors[4];" creates an array of 4 motors.
 //
 // Each motor has a "pin" on the Arduino, that outputs a PWM signal to the motor
 // to control the motor speed.
 // Each motor has its own "value" which is the speed (PWM) given to the motor.
 //
 // The four motors share ("static") information:
 //    minPWM - The minimum PWM value needed to spin the motor.
 //	  hoover - The approximate PWM value needed for the quad to take off.
 //    active - Must be true for motors to turn -- a safety lock.
 //
 //
 //	The Motor class has operations that can be performed on each motor:
 //		Off () - Turn off the motor.
 //		Set () - Set the speed (PWM) for the motor (only if Active is true).
 //
 // The four motors share some common operations:
 //		Rev () - Turn on each of the motor to maximum speed and then turn them off. The motors
 //			will be turned on and off in order: Front, Back, Left, and Right. This command is used
 //			test the motors and their wiring. If the motors do not turn on in the right order,
 //			they are wired to wrong connections. If they do not sound like they are rev-ing
 //			up to a high speed, there is either a loose connection, partially broken wire,
 //			or a bad Mosfet. If the motors do not turn off, you have a bad Mosfet.
 //		Active (status) - Turn off all the motors. If status is false, lock the motors so they
 //			cannot be turn off until Active is called with a true status.

 class Motor {
 private:

 public:
 	uint8_t pin;				// output pwm pin on Arduino
 	uint8_t value;				// current motor pwm setting
 	static uint8_t minPWM;		// minimum value to spin motor
 	static uint8_t hover;		// nominal value to hover
 	static bool active;			// active = false - motors stay off
 	static Motor motors[4];		// four motors on quad copter

 	Motor (uint8_t inPin);		// constructor
 	void Off ();				// turn off the motor
 	void Set (uint8_t pwm);		// set the motor speed
 	static void Active(bool inActive); // set motors off; inActive = false, keep motors off
 	void Rev();					// rev motors to max speed in order: Front, Back, Left, Right
 }; //end of Motor class

 // Names for the four motors
 enum Motors_t {Front, Back, Left, Right};

 // Names for the axis the the quad may be turn. Yes, Altitude is not really an axis.
 enum Axis_t   {Yaw, Pitch, Roll, Altitude}; //
 	// Yaw      - Left-Right Direction copter's nose is facing
 	// Picth    - Up-Down Direction copter's nose is facing
 	// Roll     - Right-Left Wing higher than the other
 	// Altitude - How high off the ground (not currently implemented)

 // Names for the three PID control factors.
 enum Pid_t {Proportional, Integral, Derivative};

 //
 // The Flight class tracks and controls the overall fight of the quad.
 // Only one Flight exists, because we only have one quad. Everything is accessed statically.
 //
 // attitude[4] - contains direction the quad is facing and its altitude
 // desiredAttitude[4] - is direction and altitude we want the quad facing.
 //      When the quad is level with the ground it will stay in one place.
 //		When the quad tilts (e.g., pitches the nose up), the quad will move in that direction.
 //		When the side motors turn faster or slower than the front/back motors, the quad yaws (turns).
 //		So by setting an attitude other than level, the quad will move. You are now in control!
 //
 //	PID Pids[4] - one PID for each axis (Pitch, Yaw, Roll, and Altitude).
 //
 //	What is a PID? Good question!
 //	PID stands for Proportional, Integral, and Derivative.
 //  Ok, so what does that mean? Another good question!
 //
 //	PID is a very common control mechanism. In our case we are trying the control the Pitch,
 //	Yaw, Roll, and Altitude of the quad; by setting the motor speeds.
 //	So for example, by keeping the front motor at a constant speed and changing the speed
 //	of the back motor, we control the pitch.
 //	The keeping the front and back motor speeds constant, we simultaneously add (or subtract
 //	speed to the side motors to adjust yaw. Then we can add speed to one side motor and
 //	simultaneously subtract the same speed from the other side motor, to adjust roll (left or
 //	right wing higher than the other).
 //
 //	How does changing the speed of the side motors affect yaw? The front and back motors spin in
 //	one direction (lets say clockwise). The two side motors spin in the other direction, lets say
 //	counter clockwise. Imagine, you only had one motor, lets say it was spinning its propeller
 //	clockwise. If the motor is attach to the quad and the quad is sitting on the floor, then
 //	friction would keep the quad from spinning is the opposite direction of the propeller. But
 //	once in the air, the quad would spin. On a regular helicopter there is a tail rotor
 //	(propeller) to prevent the spinning. In the same way, if all four motors on the quad spun
 //	propellers in the same direction, the quad would spin in the opposite direction. By putting
 //	two motors in one direction and the two motors in the other direction, they counter each
 //	others force to spin the quad. When we want the quad to turn (spin just a little), we speed
 //	up (or slow down) the motors going in one direction.
 //
 //	Ok, so what is PID and why are we using PID? PID is the control mechanism that tells us
 //	how much power to apply to each motor. If we want to front and back motor to apply the
 //	same amount of lift (so the quad remains level), why not give them both the same amount
 //	of power?
 //		- Remember you slapped the (heavy) battery on the bottom of the quad. Probably not
 //		    exactly in the middle, so one motor need to lift more to balance thing out. Also
 //			the wings and motors are not all glued on exactly straight.
 //		- Motors were selected to give a lot of power at an affordable price. Some motors will
 //			spin faster with the same power.
 //		- ...
 //	So we have to set the front motor based on the throttle, then adjust/twiddle with back
 //	motor to keep thing balance. With four motors to twiddle with, a person could not keep
 //	up. So PID does the twiddling.
 //
 //	PID is algorithm, a computer formula. PID has two inputs: the desired output and the
 //	current output (e.g., Pitch). PID then produces the Input (motor speed) that might
 //	make the Current Output become the Desired Output. Might? PID does not get it right the
 //	first time, but it keeps trying.
 //
 //	Why doesn't PID get it right the first time? Imagine your car is stopped. You want to know
 //	how hard to push on the gas pedal to make the car go 60 miles per hour. Well first, no matter
 //	how hard you press on the gas, your car is not going to be going 60 mph after one second. So
 //	PID would have to give the car a lot of gas to get the car going and accelerating. AS we get
 //	closer to 60, we have to ease of the gas so we don't keep on accellarating to 70, 80, ...
 //
 //	The Proportional part of PID, increases the Input based on how far we hare from the
 //	desired Output. If we are far away, add a lot; so we get there faster. If we are close,
 //	add just a little; so we don't zoom past our goal.
 //
 //	The Integral part of PID, increases the Input based on how far and how long we have been
 //	away from the desired Output. Proportional is the main factor driving us toward the
 //	desired Output. But if Proportional is not getting us there fast enough, Integral
 //	gives the extra juice to get us there faster.
 //
 //	The Derivative part of PID, controls the Input based on how fast the Output is changing.
 //	The Derivative smooths out changes.
 //
 //	The quad uses Proportion and Integral. It does not use Derivative.
 //
 // --------------------------------------------------------------------
 //
 //	Begin () - Initializes the Flight controls (turn off the motors).
 //
 //	SetThrottle(speed) - Set the speed of the front motor (and gives a quick
 //		adjust to the other motors).
 //
 //	Update(yaw, pitch, roll) - Does the PID magic and updates the motor speeds.
 //
 //	Light (light, value) - Turns on light to a specified brightness.
 //
 //	Lights (lights) - Turn on the lights in a binary pattern. Used for debug output.
 //
 //

 class Flight {
 private:

 public:
 	static double  attitude[4];			// current direction (and hieght) of copter
 	static double  desiredAttitude[4];	// setpoint direction
 	static double  previousAttitude[4];	// last pass direction
 	static PID     Pids[4];				// PIDs for each Axis_t
 	static double  motorPWM[4];			// motor speed CHANGE outputed by PID

 	static unsigned long now;			// current time
 	static unsigned long previousTime;	// time we last updated the values

 	Flight();	// default constructor

 	// user copter throttle setting
 	static void SetThrottle(uint8_t inThrottle);	// user copter throttle setting

 	// Update, new attitude each time sensors are read
 	static void Update (double yaw, double pitch, double roll, double altitude = 0);

 	// Initialize Flight (turn off motors)
 	static void Begin();

 	// Set one light's value
 	static void Light (uint8_t light, uint8_t value);

 	// Set all the lights (probably used for debugging)
 	static void Lights (uint8_t lights);

 }; // end of Flight class

 //The Arduino Digital Output pins for each of the LEDs
 #define WhiteLED   (9)
 #define BlueLED    (44)
 #define GreenLED   (45)
 #define RedLED     (46)
 #define ArduinoLED (13)
 	// ArduinoLED on the Quad

 #endif
 */

