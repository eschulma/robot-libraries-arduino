#ifndef DifferentialDriveRobot_h
#define DifferentialDriveRobot_h
#include "Arduino.h"
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

#define ROBOT_NO_VALID_DATA -1001

class DifferentialDrive {
	protected:
		ControllerMotor *motor[2];
		WheelEncoder *encoder[2];
		Odometer odom;
		StallWatcher* stallWatcher;

		int desiredWallSensorReading[2];
		float sideWallLossFactor;
		short wallSensorPin[2];
		// short wallRearSensorPin[2];

		double wheelDiameter[2];
		float trackWidth;	// units of measurement here determine it everywhere

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

		// wall following
		void followWall(short direction, float velocityFactor, int idealWallSensorReading);
		float getSideWallDistanceReading(short direction) { return analogRead(wallSensorPin[direction]); };

		// children must override these
		virtual void setup() = 0;		
	public:
		// movement
		void stop();
		void drive(int velocityLeft, int velocityRight);
		void driveTowardGoal(float velocityFactor = 1.0);
		boolean move(double x, double y, boolean inRobotFrame = true, boolean stopAfterManeuver=true);
		void setGoal(double x, double y, boolean inRobotFrame = true);
		boolean goToGoal(double goalX, double goalY, boolean stopAfterManeuver=true);
		boolean turn(double deltaHeading, boolean stopAfterManeuver=true);
		void backUp(float distance, boolean stopAfterManeuver=true);	// go this distance in a straight line, forward or back only DEPRECATED
		float recover();

		short getMaxAllowedPWM() const { return maxAllowedPWM; }
		float getMoveCalculatedPWM() const { return moveCalculatedPWM; }
		// float getBatteryChargeLevel() { return batteryChargeLevel; };
		boolean isStalled() { return stallWatcher->isStalled(); };
		void resetStallWatcher() { stallWatcher->reset(); };

		// odometry
		void updateOdometry() { odom.update(); };
		void markPosition() { odom.markPosition(); };
		float getDistanceFromMarkedPoint() { return odom.getDistanceFromMarkedPoint(); }
		long getOdometerValue(short motorIndex) { return motor[motorIndex]->getOdometerValue(); };
		// void resetOdometer(short direction) { motor[direction].resetOdometer(); };
		void resetOdometers();
		void resetCalculatedMovePWMs();
		double getX() { return odom.getX(); };
		double getY() { return odom.getY(); };
		double getHeading() { return odom.getHeading(); };
		float getTrackWidth() { return trackWidth; };

		// wall following
		void initDesiredIRSensorReadings(short direction);
		void followWall(short direction, float velocityFactor = 1.0) { 	followWall(direction, velocityFactor, desiredWallSensorReading[direction]); };
		float getFollowWallSpeed() { return followWallPWM; };
		float getMoveSpeed() { return movePWM; };
		int getSideClosestToForwardObstacle();
		float getFollowWallCalculatedPWM() const { return followWallCalculatedPWM; }
		boolean isSideWallLost(short direction);

		DifferentialDrive();

#ifdef FIREFIGHTER_TEST
		friend class RobotTester;	// for testing
#endif

};
#endif
