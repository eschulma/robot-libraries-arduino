#ifndef ControllerMotor_h
#define ControllerMotor_h 
#include <Motor.h>

/**
 *
 *	For gear motors managed by a controller with one pin for PWM speed, and another for direction.
 *
 **/

// put controller specific definitions in here
// Arduino motor shield (LEFT = A)
#define ARDUINO_MOTOR_SHIELD_LEFT_DIRECTION_PIN 12
#define ARDUINO_MOTOR_SHIELD_LEFT_ENABLE_PIN 3
#define ARDUINO_MOTOR_SHIELD_LEFT_BRAKE_PIN 9
#define ARDUINO_MOTOR_SHIELD_LEFT_CURRENT_SENSE_PIN A0
#define ARDUINO_MOTOR_SHIELD_RIGHT_DIRECTION_PIN 13
#define ARDUINO_MOTOR_SHIELD_RIGHT_ENABLE_PIN 11
#define ARDUINO_MOTOR_SHIELD_RIGHT_BRAKE_PIN 8
#define ARDUINO_MOTOR_SHIELD_RIGHT_CURRENT_SENSE_PIN A1

// Ardumotor
#define ARDUMOTOR_LEFT_ENABLE_PIN 3
#define ARDUMOTOR_RIGHT_ENABLE_PIN 11
#define ARDUMOTOR_LEFT_DIRECTION_PIN 12
#define ARDUMOTOR_RIGHT_DIRECTION_PIN 13

class ControllerMotor : public Motor {
	private:
		short directionPin;
		short brakePin;
		boolean isBrakeSet;
		
		void setMotorDirection(boolean bForward);
		void brake();
		void releaseBrake();
	public:
		ControllerMotor(short pwmEnablePin, short inDirectionPin, short inBrakePin);
		void setup();
		
		void stop();
		void setVelocity(int commandedVelocity);
				
	// friend class RobotTester;
};
#endif

