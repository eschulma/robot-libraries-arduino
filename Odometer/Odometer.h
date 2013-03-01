/**
 *	Odometer.h - Library for calculating distance and velocity from encoder counts
 *	for a differential drive system.
 *	
 *	Some of the functions here incorporate code from Dr. Rainer Hessmer. See
 *	http://code.google.com/p/drh-robotics-ros/
 *
 *  Erica S. Kane
 *	http://www.thekanes.org/
 *	
 */
#ifndef Odometer_h
#define Odometer_h

#include <WheelEncoder.h>

class Odometer
{
public:
	Odometer();
	void setup(WheelEncoder *inEncoder[2], boolean isEncoderForward[2], double inWheelDiameter[2], double trackWidth);
	void update();
	void reset();
	void delay(long ms);

	void setGoalPosition(double x, double y);
	void setGoalHeading(double inHeading) { goalHeading = inHeading; };

	void transformRobotPointToOdomPoint(double *x, double *y);
	// used for remote control
	void translateToLeftRightVelocities(float *newLeft, float *newRight, float commandedVelocity, float commandedAngularVelocity);

	double getHeading() { return heading; };
	double getGoalHeading() { return goalHeading; };
	double calculateGoalHeading();
	double getHeadingError();
	double getHeadingError(double goalHeading);
	double getDistanceToGoal();
	double getLinearVelocity();
	double getAngularVelocity() const { return omega; };

	double getGoalX() const { return goalX; }
	double getGoalY() const { return goalY; }
	double getX() const { return X; }
	double getY() const { return Y; }
//	double getVLeft() const { return vLeft; };
//	double getVRight() const { return vRight; };

private:
	WheelEncoder *encoder[2];
	int encodeFactor[2];
	double wheelDiameter[2];
	double trackWidth;
	double countsPerRevolution[2];
	double distancePerCount[2];
	// double turnFudgeFactor;

	long previousLeftEncoderCounts;
	long previousRightEncoderCounts;
	unsigned long previousUpdateTime;

	double goalX;
	double goalY;
	double goalHeading;

	// filled by update
	double X;  // x coord in global frame
	double Y;  // y coord in global frame
	double heading;  // heading (radians) in the global frame. The value lies in (-PI, PI]
	double vLeft;
	double vRight;
	double omega;

	void setCurrentPosition(double x, double y, double heading);
	void calculateDeltasCrude(double *deltaX, double *deltaY,
			double deltaDistanceLeft, double deltaDistanceRight);
	void calculateDeltasRefined(double *deltaX, double *deltaY,
			double deltaDistanceLeft, double deltaDistanceRight);
};
#endif
