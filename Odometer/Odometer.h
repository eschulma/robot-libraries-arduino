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

	void setGoalPosition(double x, double y);
	void setTargetHeading(double inHeading) { targetHeading = inHeading; }; // used for turn function
	void markPosition();
	void markPosition(double x, double y);

	void setCurrentPosition(double x, double y, double heading);

	void transformRobotPointToOdomPoint(double *x, double *y);
	static void transformPoint(double *x, double *y, double deltaX, double deltaY, double deltaHeading);

	// used for remote control
	void translateToLeftRightVelocities(float *newLeft, float *newRight, float commandedVelocity, float commandedAngularVelocity);

	double getHeading() { return heading; };
	double getTargetHeading() { return targetHeading; }; // used for turn function
	double calculateGoalHeading();
	double getHeadingError();
	double getHeadingError(double goalHeading);

	static double getDistanceBetweenPoints(double x1, double y1, double x2, double y2);
	double getDistanceToGoal();
	double getDistanceFromMarkedPoint();

	double getLinearVelocity();
	double getAngularVelocity() const { return omega; };

	double getGoalX() const { return goalX; }
	double getGoalY() const { return goalY; }
	double getX() const { return X; }
	double getY() const { return Y; }
	double getMarkX() const { return markX; }
	double getMarkY() const { return markY; }
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
	long maxUpdateTime;

	double goalX;
	double goalY;
	double markX;
	double markY;
	double targetHeading;

	// filled by update
	double X;  // x coord in global frame
	double Y;  // y coord in global frame
	double heading;  // heading (radians) in the global frame. The value lies in (-PI, PI]
	double vLeft;
	double vRight;
	double omega;

	void calculateDeltasCrude(double *deltaX, double *deltaY,
			double deltaDistanceLeft, double deltaDistanceRight);
	void calculateDeltasRefined(double *deltaX, double *deltaY,
			double deltaDistanceLeft, double deltaDistanceRight);
};
#endif
