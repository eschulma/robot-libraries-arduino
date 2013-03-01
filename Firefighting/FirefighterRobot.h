#ifndef FirefighterRobot_h
#define FirefighterRobot_h
#include <Arduino.h>
#include <ControllerMotor.h>
#include <Motor.h>
#include <WheelEncoder.h>
#include <math.h>
#include <Ultrasonic.h>
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
		Ultrasonic *sonar[5];
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

		int moveSpeed;
		int turnSpeed;
		int followWallSpeed;
		float targetMoveVelocity;
		float targetFollowVelocity;
		float targetAngularVelocity;
		int bangBangDelta;
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
		
		int panServoForFire(int startDegree, int endDegree, boolean bNearby);
		
		float getFireReading();
		
		// children must override these
		virtual void setup() = 0;		
	public:
		Odometer odom;

		void stop();
		void drive(int velocityLeft, int velocityRight);
		boolean move(double forward, double left, boolean resetWorldFrameToRobot = true);
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
		
		// determines if we do PID or bang-bang
		void followWall(short direction, int idealWallSensorReading);

		void initDesiredWallSensorReadings(short direction);
		void followWall(short direction) { 	followWall(direction, desiredWallSensorReading[direction]); };	
		void followWallRear(short direction) { 	followWallRear(direction, desiredWallSensorReading[direction]); };	
		void followWallRear(short direction, int idealWallSensorReading);
		float getFollowWallSpeed() { return followWallSpeed; };
		float getMoveSpeed() { return moveSpeed; };
		
		boolean isSideWallLost(short direction);
		float getFrontWallDistance() { float value = getSonarWallDistance(SONAR_FRONT); return value; }
		float getSideWallDistance(short direction);
		
		void alignLeft() { align(ROBOT_LEFT); };
		void alignRight() { align(ROBOT_RIGHT); };

		long getOdometerValue(short motorIndex) { return motor[motorIndex]->getOdometerValue(); };
		// void resetOdometer(short direction) { motor[direction].resetOdometer(); };
		void resetOdometers() { odom.reset(); };

		float getTrackWidth() { return trackWidth; };

		FirefighterRobot();
		
		friend class RobotTester;	// for testing
};
#endif