#include "DifferentialDrive.h"

DifferentialDrive::DifferentialDrive() {
	// set everything to blank or default values (but not motor or encoder!!)
	trackWidth = 0;

	for(int i = 0; i < 2; i++) {
		desiredWallSensorReading[i] = 340;
	}

	movePWM = 0.0f;
	turnPWM = 0.0f;
	followWallPWM = 0.0f;

	// default
	moveMotorIndex = MOTOR_LEFT;	// motor whose encoder will dominate

	rightSideSlowFactor = 1.0;
	leftSideSlowFactor = 1.0;
	turnOneWheelOnly = true;
	turnFudgeFactor = 1.5;	
}

void DifferentialDrive::stop() {
	drive(0,0);
	motor[MOTOR_LEFT]->stop();
	motor[MOTOR_RIGHT]->stop();

	// new -- to avoid lurches
	resetCalculatedMovePWMs();
}

/**
 *	Assumes that equal velocities will give a straight motion, more or less. If that's not the
 *  case, a child class can override this by setting rightSideSlowFactor.
 */
void DifferentialDrive::drive(int velocityLeft, int velocityRight) {
	// to avoid delays
	float vl = velocityLeft * leftSideSlowFactor;
	float vr = velocityRight * rightSideSlowFactor;

	motor[MOTOR_LEFT]->setVelocity(vl);
	motor[MOTOR_RIGHT]->setVelocity(vr);
}

/**
 * Move the robot forward and left the specified amount.
 * We don't care about the final heading.
 *
 * By default, we assume x and y are defined in the robot frame,
 * but one can set inRobotFrame to false to use the global
 * odometry frame instead.
 *
 * Return true for success, false for failure.
 */
boolean DifferentialDrive::move(double x, double y, boolean inRobotFrame, boolean stopAfterManeuver) {
	if(inRobotFrame) {
		// must transform from robot frame to world frame
		odom.transformRobotPointToOdomPoint(&x, &y);
	}

	// now x and y are in the world frame
	return goToGoal(x, y, stopAfterManeuver);
}

/**
 * Set odometry goal, by default in the robot frame.
 */
void DifferentialDrive::setGoal(double x, double y, boolean inRobotFrame) {
	if(inRobotFrame) {
		// must transform from robot frame to world frame
		odom.transformRobotPointToOdomPoint(&x, &y);
	}

	// now x and y are in the world frame
	odom.setGoalPosition(x, y);
	return;
}



/**
 * Move the robot to the specified goal point in world coordinates.
 * We don't care about the final heading. If you want to do this in
 * robot coordinates, use move instead.
 *
 * This is a little risky to call repeatedly, unless the odometry is very
 * accurate or has been corrected by sensor input. If it isn't, the world
 * and robot frames will disconnect fairly quickly.
 *
 * Caller is responsible for calling stop() after this function, if that is the
 * desired behavior, otherwise the bot will keep going.
 *
 * Return true for success, false for failure.
 */
boolean DifferentialDrive::goToGoal(double goalX, double goalY, boolean stopAfterManeuver) {
	boolean bSuccess = false;

	odom.setGoalPosition(goalX, goalY);
	odom.update();

	Serial.print("Goal heading (degrees): ");
	Serial.println(odom.calculateGoalHeading() * RAD_TO_DEG);
	Serial.print("Current heading (degrees): ");
	Serial.println(odom.getHeading() * RAD_TO_DEG);
//	Serial.print("Heading error (degrees): ");
//	Serial.println(odom.getHeadingError() * RAD_TO_DEG);

	float maxDistanceError = 1;
	float minDistance = 10000;
	float distanceToGoal;
	float distanceNoiseMargin = 3;

	boolean wasSlowDownPerformed = false;
	float velocityFactor = 1.0;
	float slowDownFactor = 0.5;
	float slowDownDistance = 1;

	stallWatcher->reset();

	// Turn towards goal first, otherwise for short distance and
	// big turn required, we may have issues.
	if(fabs(odom.getHeadingError()) > 5 * DEG_TO_RAD) {
		Serial.println("Turning towards goal.");
		turn(-odom.getHeadingError());
		stop();
		delay(500);
	}

	resetOdometers();
	odom.update();

	// make sure we start at a reasonable speed
	resetCalculatedMovePWMs();

	do {
		driveTowardGoal(velocityFactor);

		distanceToGoal = odom.getDistanceToGoal();
//		Serial.print("Distance to goal:  ");
//		Serial.println(distanceToGoal);

		if(!wasSlowDownPerformed) {
			if(distanceToGoal < slowDownDistance) {
				velocityFactor *= slowDownFactor;
				wasSlowDownPerformed = true;
				// Serial.println("Slowing down.");
			}
		}

		// allow for some noise here
		if(distanceToGoal < minDistance) {
			minDistance = distanceToGoal;
		}
		else if(distanceToGoal > minDistance + distanceNoiseMargin) {
			// we overshot!
			Serial.println("Stopping due to overshoot.");
			Serial.print("minDistance: ");
			Serial.println(minDistance);

			// we still succeeded
			bSuccess = true;
			break;
		}

		// never give up before we've reached full power
		if((stallWatcher->isStalled()) && (fabs(moveCalculatedPWM) < maxAllowedPWM)) {
			stallWatcher->reset();
		}
	} while((!stallWatcher->isStalled()) && (distanceToGoal > maxDistanceError));

	Serial.print("Distance to goal: ");
	Serial.println(distanceToGoal);

	if(distanceToGoal <= maxDistanceError) {
		bSuccess = true;
	}

	if(stopAfterManeuver) {
		stop();
	}

	return bSuccess;
}

void DifferentialDrive::driveTowardGoal(float velocityFactor) {
	static long lastUpdate = 0;

	// don't let this be called too often -- we need some time to get a
	// proper velocity sample
	long currentTime = millis();
	if(currentTime - lastUpdate < 10) {
		return;
	}
	lastUpdate = currentTime;

	float targetVelocity = targetMoveVelocity * velocityFactor;	// cm per sec
	int minPWM = 1;
	int maxPWM = 254;

	// PID values. One set for heading error and one set for total velocity
	double kP = 16;
	double kP_velocity = 0.01;

	double headingError = odom.getHeadingError();	// this will be between 1 and -1

	int dlRaw = (int)ceil(moveCalculatedPWM * (1 + (kP * headingError)));
	int drRaw = (int)ceil(moveCalculatedPWM * (1 - (kP * headingError)));

	// don't allow drive to go from positive to negative...bad for motors; and 254 is max
	drive(constrain(dlRaw, minPWM, maxPWM), constrain(drRaw, minPWM, maxPWM));
	odom.update();

	// figure out the right drive voltage to use for our desired speed
	double velocityError = fabs(odom.getLinearVelocity()) - fabs(targetVelocity); // positive if we are too fast, negative for slow

//	Serial.print("Velocity error: ");
//	Serial.println(velocityError);

	moveCalculatedPWM *= (1 - (kP_velocity * velocityError));
	moveCalculatedPWM = constrain(moveCalculatedPWM, minPWM, maxPWM); // mainly needed for debugging
}

/**
* Turn in place until we have changed heading the specified number of radians. Return true for
* success. User must call stop() when this is done.
*/
boolean DifferentialDrive::turn(double headingChange, boolean stopAfterManeuver) {
	// safeguard against backwards turn
	headingChange = atan2(sin(headingChange), cos(headingChange));

	double goalHeading = headingChange + odom.getHeading();
	goalHeading = atan2(sin(goalHeading), cos(goalHeading));

	boolean bSuccess = false;

	float basePWM = turnPWM;	// units from 0 to 254, input for drive command
	float targetVelocity = targetAngularVelocity;	// rad per sec

	// for positive angle turn, right wheel is positive velocity, left is negative
	// int minLPWM = -254;
	// int maxLPWM = 0;
	int minRPWM = 0;
	int maxRPWM = 254;

	// turn in the specified direction
	if(headingChange < 0) {
		basePWM *= -1;
		targetVelocity *= -1;

		// minLPWM = 0;
		// maxLPWM = 254;
		minRPWM = -254;
		maxRPWM = 0;
	}

	double maxAngleError = 1 * DEG_TO_RAD;
	double minAngularDistance = 10000;
	double radsFromGoalAngle = 0.0;
	double angularNoiseMargin = 0.02;
	double velocityError = 0.0;

	boolean wasSlowDownPerformed = false;
	double slowDownFactor = 0.5;
	double slowDownAngularDistance = 1 * DEG_TO_RAD;

	// PID values
	double kP_accel = 2;

	odom.setTargetHeading(goalHeading);
	resetOdometers();
	odom.update();
	stallWatcher->reset();
	delay(10);

	float dr = basePWM;
	float dl = -dr;
	double accel = 0;
	int nLoops = 0;

	do {
		// acceleration is the value that works with PID
		accel = velocityError * kP_accel;	// this is negative when we are too fast
		if(headingChange < 0) {
			accel *= -1;
		}

		// be careful rounding -- the change can go to zero!
		dr += accel;

		// don't allow drive to go from positive to negative...bad for motors; and 254 is max
		dr = constrain(dr, minRPWM, maxRPWM);
		dl = -dr;

		drive(dl, dr);
		odom.update();

		// figure out the right drive voltage to use for our desired speed
		if(nLoops > 1) {
			velocityError =  fabs(targetVelocity)- fabs(odom.getAngularVelocity());
		}

//		 Serial.print("dl = ");
//		 Serial.println(dl);
//		 Serial.print("velocity error = ");
//		 Serial.println(velocityError);
//		 Serial.print("accel = ");
//		 Serial.println(accel);

		radsFromGoalAngle = fabs(odom.getHeadingError(goalHeading));
		// Serial.print("Radians from goal: ");
		// Serial.println(radsFromGoalAngle);

		if(!wasSlowDownPerformed) {
			if(radsFromGoalAngle < slowDownAngularDistance) {
				targetVelocity *= slowDownFactor;
				wasSlowDownPerformed = true;
				// Serial.println("Slowing down.");
			}
		}

		// Potentially problematic if user specifies a "backwards" turn, so put in safeguards
		// allow for some noise here
		if(radsFromGoalAngle < minAngularDistance) {
			minAngularDistance = radsFromGoalAngle;
		}
		else if(radsFromGoalAngle > minAngularDistance + angularNoiseMargin) {
			// we overshot!
			Serial.println("Stopping due to overshoot.");
			Serial.print("minAngularDistance: ");
			Serial.println(minAngularDistance);

			// we still succeeded
			bSuccess = true;
			break;
		}

		// sampling rate matters quite a bit here
		delay(10);

		// never give up before we've reached full power
		if(stallWatcher->isStalled()) {
			// Serial.print("Stalled at dl ");
			// Serial.println(dl);
			if(fabs(dl) < 250) {
				// Serial.println("Still trying...");
				stallWatcher->reset();
			}
			else {
				Serial.println("Turn stalled.");
			}
		}

		nLoops++;
	} while((!stallWatcher->isStalled()) && (radsFromGoalAngle > maxAngleError));

//	Serial.print("Radians to goal: ");
//	Serial.println(radsFromGoalAngle);

	if(radsFromGoalAngle <= maxAngleError) {
		bSuccess = true;
	}

	if(stopAfterManeuver) {
		stop();
	}

	return bSuccess;
}

/**
 *	Move the robot a short distance straight back, based on the encoder value
 *	for one motor. There is no error correction if the robot drifts to the side.
 *	Both positive and negative input values move the robot backwards!
 *
 *	The caller is responsible for calling stop() after the function, otherwise the bot
 *	will keep moving.
 *
 *	NOT RECOMMENDED IF YOU NEED ACCURACY. Use goToGoal instead. But this is useful for
 *	recovery behavior.
 **/
void DifferentialDrive::backUp(float distance, boolean stopAfterManeuver) {
	// convert distance to encoder ticks
	// distance = ticks * wheel circumference * (1 / ticks per revolution), or
	// ticks = distance * ticks per revolution / wheel circumference
	long ticks = fabs(floor( (distance * encoder[ moveMotorIndex ]->getCountsPerRevolution()) / 
		(wheelDiameter[ moveMotorIndex ] * M_PI) ));

	// we always move backwards
	float basePWM = -movePWM;
	
	resetOdometers();	// also does odom reset
	stallWatcher->reset();
	odom.update();
	long lastUpdateTime = millis();
	long currentTime;

	do {
		// no PID
		drive(basePWM, basePWM);

		currentTime = millis();
		if(currentTime - lastUpdateTime > 10) {
			odom.update();
			lastUpdateTime = currentTime;
		}
	} while((!stallWatcher->isStalled()) && (getOdometerValue(moveMotorIndex) < ticks) &&  
			(getOdometerValue(moveMotorIndex) > -ticks));

	if(stopAfterManeuver) {
		stop();
	}
}

/**
 *	Set the distance we want to maintain from wall. Should be lined up
 *	nicely when this is called. Use the sensor on the specified side to
 *	set the value.
 */
void DifferentialDrive::initDesiredIRSensorReadings(short direction) {
	long wallDistance = 0;
	for(int j = 0; j < 5; j++) {
		wallDistance += getSideWallDistanceReading(direction);
	}
	wallDistance = floor((float)wallDistance/5.0);
	desiredWallSensorReading[direction] = wallDistance;

	Serial.print("desiredWallSensorReading[");
	Serial.print(direction);
	Serial.print("]: ");
	Serial.println(	desiredWallSensorReading[direction] );

	short otherSide = 1;
	if(direction == 1) {
		otherSide = 0;
	}

	// assume the other sensor needs the same value -- testing indicates this is reasonable
	desiredWallSensorReading[otherSide] = desiredWallSensorReading[direction];
}

/**
 * Useful if the front sonar has detected an obstacle. Generally we will want to
 * turn towards the IR sensor that is furthest away. If they are tied, this function
 * returns NO_VALID_DATA.
 */
int DifferentialDrive::getSideClosestToForwardObstacle() {
	long wallReading[2];
	for(int i = 0; i < 2; i++) {
		wallReading[i] = getSideWallDistanceReading(i);
	}

	int closeSide = ROBOT_LEFT;
	if(wallReading[ROBOT_RIGHT] > wallReading[ROBOT_LEFT]) {
		closeSide = ROBOT_RIGHT;
	}
	else if(wallReading[ROBOT_RIGHT] == wallReading[ROBOT_LEFT]) {
		closeSide = ROBOT_NO_VALID_DATA;
	}

	Serial.print("Side closest to wall: ");
	Serial.println(closeSide);
	return closeSide;
}

/**
 *	The key to everything. 
 */
void DifferentialDrive::followWall(short direction, float velocityFactor, int optimalWallSensorReading) {
	static long lastUpdate = 0;

	// don't let this be called too often -- we need some time to get a
	// proper velocity sample
	long currentTime = millis();
	if(currentTime - lastUpdate < 10) {
		return;
	}

	lastUpdate = currentTime;
	float targetVelocity = targetFollowVelocity * velocityFactor;
	int drivePWM[2];

	// PID values. One set for heading error and one set for total velocity
	float kP = 0.02;
	float kP_velocity = 0.01;

	short oppositeDirection = !direction;	// this works for right and left only, note
	
	// Standard analog read here is slow!!!
	// note that IR reading is higher for closer objects
	int wallDistance = (int)getSideWallDistanceReading(direction);
	int headingError = wallDistance - desiredWallSensorReading[direction];

	drivePWM[direction] = followWallCalculatedPWM * (1 + (kP * headingError));
	drivePWM[oppositeDirection] = followWallCalculatedPWM * (1 - (kP * headingError));

	// we only go forward here; error is positive if we are too fast
	odom.update();
	float velocityError = odom.getLinearVelocity() - targetVelocity;
	followWallCalculatedPWM *= (1 - (kP_velocity * velocityError));
	
//	Serial.print("Velocity error: ");
//	Serial.println(velocityError);

	// let's go
	drive(constrain(drivePWM[MOTOR_LEFT], 0, 254), constrain(drivePWM[MOTOR_RIGHT], 0, 254));
}

boolean DifferentialDrive::isSideWallLost(short direction) {
	// ref: http://www.phidgets.com/products.php?product_id=3521
	// float distanceVoltage = getSideWallDistanceReading( direction ) * 0.0049;

	long wallDistance = getSideWallDistanceReading(direction);

	float minDistance = desiredWallSensorReading[direction] * sideWallLossFactor;
	if(wallDistance < minDistance) {
		// double check -- ESK 2/7/13
		wallDistance = getSideWallDistanceReading(direction);
		if(wallDistance < minDistance) {
			Serial.print("Wall lost at ");
			Serial.println(getSideWallDistanceReading(direction));
			return true;
		}
	}
	return false;
}

void DifferentialDrive::resetCalculatedMovePWMs() {
	moveCalculatedPWM = movePWM;
	followWallCalculatedPWM = followWallPWM;
}

/**
 * This also resets the stall watcher, an important piece
 */
void DifferentialDrive::resetOdometers() {
	odom.reset();
	stallWatcher->reset();
}

float DifferentialDrive::recover() {
	stop();
	Serial.println("Attempting recovery.");

	float angleTurned = 0;
	resetStallWatcher();

	short closeSide = getSideClosestToForwardObstacle();
	if(closeSide == ROBOT_NO_VALID_DATA) {
		// try again
		closeSide = getSideClosestToForwardObstacle();
		if(closeSide == ROBOT_NO_VALID_DATA) {
			Serial.println("Angled forward sensors useless, guessing.");
			closeSide = ROBOT_LEFT;
		}
	}

	if(closeSide == ROBOT_RIGHT) {
		turn(15.0 * DEG_TO_RAD);
		angleTurned += 15 * DEG_TO_RAD;
	}
	else {
		turn(-15.0 * DEG_TO_RAD);
		angleTurned -= 15 * DEG_TO_RAD;
	}
	stop();
	resetStallWatcher();

	return angleTurned;
}
