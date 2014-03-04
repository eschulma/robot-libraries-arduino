#ifndef FireCheetah_h
#define FireCheetah_h
#include "Arduino.h"
#include <DifferentialDrive.h>
#include <FirefightingRobot.h>
#include <math.h>

// Arduino motor shield: 3, 8, 9, 11, 13, A0, A1
// Ardumoto: pins 3, 11, 12, 13
#define WALL_LEFT_SENSOR_PIN A14
#define WALL_RIGHT_SENSOR_PIN A15
#define SONAR_FORWARD_TRIGGER_PIN 22
#define SONAR_FORWARD_ECHO_PIN A3
#define SONAR_LEFT_F_TRIGGER_PIN 23
#define SONAR_LEFT_F_ECHO_PIN A9
#define SONAR_LEFT_R_TRIGGER_PIN 24
#define SONAR_LEFT_R_ECHO_PIN A10
#define SONAR_RIGHT_F_TRIGGER_PIN 25
#define SONAR_RIGHT_F_ECHO_PIN A6
#define SONAR_RIGHT_R_TRIGGER_PIN 26 
#define SONAR_RIGHT_R_ECHO_PIN A7

// #define FIRE_SENSOR_PIN A8
#define SERVO_CONTROL_PIN 10
#define FAN_CONTROL_PIN 27

#define START_BUTTON_PIN 2
#define AUDIO_START_PIN A11
// note battery monitor not used now, this pin is in use
#define BATTERY_MONITOR_PIN A15

// For Mega, connect the SDA and SCL pins to 20 and 21

// for encoders, A pin is yellow
// TODO: Change left encoder pins
#define ENCODER_LEFT_A_PIN 20
#define ENCODER_LEFT_B_PIN 21
#define ENCODER_RIGHT_A_PIN 18
#define ENCODER_RIGHT_B_PIN 19

#define NO_SONAR_FRONT_WALL_SEEN 1000

enum sonarLocation {
	SONAR_FRONT = 0,
	SONAR_LEFT_F = 1,
	SONAR_LEFT_R = 2,
	SONAR_RIGHT_F = 3,
	SONAR_RIGHT_R = 4
};

class FireCheetah : public DifferentialDrive, public FirefightingRobot {
	private:
		// sonar
		NewPing *sonar[5];

		// sonar functions
		float getSonarDistance(sonarLocation index);
		float getMisalignment(short direction);

		// fire
		boolean bSetupFireSensor;
	public:
		FireCheetah(boolean setupFireSensor = true);
		void setup();
		void fightFire(int initDegrees);

		// sonar functions
		float getFrontWallDistance();
		float getSideWallDistance(short direction); // NOTE: this is slow!
		boolean isAlignmentPossible(short direction, float maxDistance);
		float getMisalignmentAngle(short direction);
		boolean isWayForwardBlocked();
		boolean align(short direction);
		float getCalculatedWallEnd(short direction);
		boolean isSideWallPresent(short direction);

#ifdef FIREFIGHTER_TEST
		friend class RobotTester;	// for testing
#endif
};
#endif
