#ifndef FirefighterRobot_h
#define FirefighterRobot_h
#include <Arduino.h>
#include <ControllerMotor.h>
#include <Motor.h>
#include <WheelEncoder.h>
#include <math.h>
#include <NewPing.h>
#include <StallWatcher.h>
#include <Servo.h>
#include <Odometer.h>

#define ROBOT_LEFT 0
#define ROBOT_RIGHT 1
#define ROBOT_FRONT 2
#define ROBOT_REAR 3

#define ROBOT_NO_FIRE_FOUND -1000
#define ROBOT_NO_VALID_DATA -1001

enum sonarLocation {
	SONAR_FRONT = 0,
	SONAR_LEFT_F = 1,
	SONAR_LEFT_R = 2,
	SONAR_RIGHT_F = 3,
	SONAR_RIGHT_R = 4
};

class FirefighterRobot {
	protected:
		ControllerMotor *motor[2];
		WheelEncoder *encoder[2];
		NewPing *sonar[5];
		StallWatcher* stallWatcher;
		Servo fanServo;

		int desiredWallSensorReading[2];
		float sideWallLossFactor;
		float frontWallFoundDistance;
		float rearWallFoundDistance;
		short fanServoDegrees;
		
		short wallSensorPin[2];
		short wallRearSensorPin[2];
		short fireSensorPin;
		short fanControlPin;
		
		double wheelDiameter[2];
		float trackWidth;	// units of measurement here determine it everywhere
		
		float batteryChargeLevel;	

		short maxAllowedPWM;
		short movePWM;
		short turnPWM;
		short followWallPWM;

		// keep these floating point, important
		float followWallCalculatedPWM;
		float moveCalculatedPWM;

		float targetMoveVelocity;
		float targetFollowVelocity;
		float targetAngularVelocity;
		short moveMotorIndex;

		boolean turnOneWheelOnly;
		float turnFudgeFactor;
		float rightSideSlowFactor;
		float leftSideSlowFactor;
		short fireThresholdReading;
		short fireThresholdReadingNear;
		
		float getSideWallDistanceReading(short direction) { return analogRead(wallSensorPin[direction]); };
		float getSideWallRearDistanceReading(short direction) { return analogRead(wallRearSensorPin[direction]); };
		float getSonarWallDistance(sonarLocation index);		
		
		boolean align(short direction);
		float getMisalignment(short direction);
		
		void followWall(short direction, float velocityFactor, int idealWallSensorReading);

		int panServoForFire(int startDegree, int endDegree, boolean bNearby);
		
		float getFireReading();
		
		// children must override these
		virtual void setup() = 0;		
	public:
		Odometer odom;

		void stop();
		void drive(int velocityLeft, int velocityRight);
		void driveTowardGoal(float velocityFactor = 1.0, boolean goBackwards = false);
		boolean move(double x, double y, boolean inRobotFrame = true);
		boolean goToGoal(double goalX, double goalY);
		boolean turn(double deltaHeading);
		void move(float distance);	// go this distance in a straight line, forward or back only DEPRECATED

		void setFanServo(short degrees);	// 0 is pointing forward
		void turnFanOn(boolean on);
		int panServoForFire();
		void fightFire();		
		void recover();

		boolean isFire(boolean bNear);		
		float getBatteryChargeLevel() { return batteryChargeLevel; };
		// StallWatcher *getStallWatcher() { return stallWatcher; };
		boolean isStalled() { return stallWatcher->isStalled(); };
		void resetStallWatcher() { stallWatcher->reset(); };
		
		void initDesiredWallSensorReadings(short direction);
		void followWall(short direction, float velocityFactor = 1.0) { 	followWall(direction, velocityFactor, desiredWallSensorReading[direction]); };
		void followWallRear(short direction) { 	followWallRear(direction, desiredWallSensorReading[direction]); };	
		void followWallRear(short direction, int idealWallSensorReading);
		float getFollowWallSpeed() { return followWallPWM; };
		float getMoveSpeed() { return movePWM; };
		float getIRWallForwardDistance(short direction);
		
		boolean isSideWallLost(short direction);
		float getFrontWallDistance() { float value = getSonarWallDistance(SONAR_FRONT); return value; }
		float getSideWallDistance(short direction);
		float getMisalignmentAngle(short direction);
		
		void alignLeft() { align(ROBOT_LEFT); };
		void alignRight() { align(ROBOT_RIGHT); };

		long getOdometerValue(short motorIndex) { return motor[motorIndex]->getOdometerValue(); };
		// void resetOdometer(short direction) { motor[direction].resetOdometer(); };
		void resetOdometers() { odom.reset(); };
		void resetCalculatedMovePWMs();

		float getTrackWidth() { return trackWidth; };
		float getFollowWallCalculatedSpeed() { return followWallCalculatedPWM; };

		FirefighterRobot();

#ifdef FIREFIGHTER_TEST
		friend class RobotTester;	// for testing
#endif
};
#endif
