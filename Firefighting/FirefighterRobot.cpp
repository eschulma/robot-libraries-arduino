#include "FirefighterRobot.h"

FirefighterRobot::FirefighterRobot() {
	// set everything to blank or default values (but not motor or encoder!!)
	trackWidth = 0;

	for(int i = 0; i < 2; i++) {
		desiredWallSensorReading[i] = 340;
	}

	// consider using a potentiometer for setting these
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

void FirefighterRobot::stop() {
	drive(0,0);
	motor[MOTOR_LEFT]->stop();
	motor[MOTOR_RIGHT]->stop();
}

/**
 *	Assumes that equal velocities will give a straight motion, more or less. If that's not the
 *  case, a child class can override this by setting rightSideSlowFactor.
 */
void FirefighterRobot::drive(int velocityLeft, int velocityRight) {
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
boolean FirefighterRobot::move(double x, double y, boolean inRobotFrame) {
	if(inRobotFrame) {
		// must transform from robot frame to world frame
		odom.transformRobotPointToOdomPoint(&x, &y);
	}

	// now x and y are in the world frame
	return goToGoal(x, y);
}

/**
 * Set odometry goal, by default in the robot frame.
 */
void FirefighterRobot::setGoal(double x, double y, boolean inRobotFrame) {
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
boolean FirefighterRobot::goToGoal(double goalX, double goalY) {
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

	// Turn towards goal first, otherwise for short distance and
	// big turn required, we may have issues.
	if(fabs(odom.getHeadingError()) > 5 * DEG_TO_RAD) {
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

	return bSuccess;
}

void FirefighterRobot::driveTowardGoal(float velocityFactor) {
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
* success.
*/
boolean FirefighterRobot::turn(double headingChange) {
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
		}

		nLoops++;
	} while((!stallWatcher->isStalled()) && (radsFromGoalAngle > maxAngleError));

//	Serial.print("Radians to goal: ");
//	Serial.println(radsFromGoalAngle);

	if(radsFromGoalAngle <= maxAngleError) {
		bSuccess = true;
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
void FirefighterRobot::backUp(float distance) {
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
}

void FirefighterRobot::turnFanOn(boolean on) {
	if(on) {
		digitalWrite(fanControlPin, HIGH);
	}
	else {
		digitalWrite(fanControlPin, LOW);
	}
}

/**
 *	Set the distance we want to maintain from wall. Should be lined up
 *	nicely when this is called. Use the sensor on the specified side to
 *	set the value.
 */
void FirefighterRobot::initDesiredIRSensorReadings(short direction) {
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
int FirefighterRobot::getSideClosestToForwardObstacle() {
	long wallReading[2];
	for(int i = 0; i < 2; i++) {
		wallReading[i] = getSideWallDistanceReading(i);
		// IR sensors do seem to return 0 when nothing seen...complicates things
		if(wallReading[i] == 0) {
			wallReading[i] = 10000;
		}
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
void FirefighterRobot::followWall(short direction, float velocityFactor, int optimalWallSensorReading) {
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

boolean FirefighterRobot::isSideWallLost(short direction) {
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

/**
 * CM from nearest obstacle for this sonar.
 * This will return ROBOT_NO_VALID_DATA for distances greater than max distance -- be careful.
 */
float FirefighterRobot::getSonarDistance(sonarLocation loc) {
	// Serial.print("Getting distance for sonar wall ");
	// Serial.println(loc);

	float wallDistance = sonar[loc]->ping_cm();
	//Serial.print("Wall distance ");
	//Serial.println(wallDistance);
	
	if(wallDistance == 0) {
		wallDistance = ROBOT_NO_VALID_DATA;
	}

	return wallDistance;
}

/**
 *	Returns the cm by which the left front sensor is farther away than the left
 *	rear sensor. After that basic trigonometry can be used to align the robot with the left wall.
 *
 *	For the right, we reverse front and rear.
 */
float FirefighterRobot::getMisalignment(short direction) {
	boolean readingFailed = false;

	sonarLocation front = SONAR_LEFT_F;
	sonarLocation rear = SONAR_LEFT_R;
	if(direction == ROBOT_RIGHT) {
		front = SONAR_RIGHT_R;
		rear = SONAR_RIGHT_F;
	}

	float distance1 = 0;
	
	// Filter out bad values. NewPing automatically returns 0 for distances past max distance set in
	// constructor
	short numRetries = 0;
	do {
		delay(100);
		distance1 = sonar[front]->ping_cm();
		numRetries++;
	} while((distance1 < 0.01) && (numRetries < 5));

	if(distance1 < 0.01) {
		readingFailed = true;
	}
	
	float distance2 = 0;
	if(!readingFailed) {
		delay(100);

		numRetries = 0;
		do {
			delay(100);
			distance2 = sonar[rear]->ping_cm();
			numRetries++;
		} while((distance2 < 0.01) && (numRetries < 5));
		if(distance2 < 0.01) {
			readingFailed = true;
		}
	}
	
	Serial.print("distance1 ");
	Serial.print(distance1);
	Serial.print(" distance2 ");
	Serial.println(distance2);
	
	if(readingFailed) {
		return ROBOT_NO_VALID_DATA;
	}
	
	return (distance1-distance2);
}

/**
 *	Line up robot parallel to wall. We could use the boolean to say if this was
 *	possible or successful. Right now, no safety checks.
 */
boolean FirefighterRobot::align(short direction) {
	boolean bSuccess = false;

	Serial.print("Aligning robot, side ");
	Serial.println(direction);

	float theta = getMisalignmentAngle(direction);

	if(theta != ROBOT_NO_VALID_DATA) {
		if(fabs(theta) > 2 * DEG_TO_RAD) {
			bSuccess = turn(theta);
		}
		else {
			Serial.println("Unable to get misalignment angle, doing nothing.");
		}
	}

	stop();
	return bSuccess;
}

/**
 * What value do we need to plug into turn in order to get the robot
 * facing parallel to the wall. Note the sign will be different on different
 * sides.
 *
 * Returns ROBOT_NO_VALID_DATA if failed.
 */
float FirefighterRobot::getMisalignmentAngle(short direction) {
	float sensorSpacing = 7.7;

 	float y = getMisalignment(direction);
 	
 	if(y == ROBOT_NO_VALID_DATA) {
 		Serial.println("Unable to get good data for alignment.");
 		return ROBOT_NO_VALID_DATA;
 	}

//  Serial.print("Raw diff: ");
//  Serial.println(y);

	// got back garbage from diff function
 	if(fabs(y) > 160) {
 		Serial.print("getMisalignment returned too high a value: ");
 		Serial.println(y);
 		return ROBOT_NO_VALID_DATA;
 	}
  
 	float theta = asin(y/sensorSpacing);
 	theta = atan2(sin(theta), cos(theta));
  
//  Serial.print("diff: ");
//  Serial.println(y);
//  Serial.print("Raw theta: ");
//  Serial.println(theta);
      
//  if(fabs(y) > sensorSpacing) {
//  	// more than 45 degrees off
//  	Serial.println("Aligning more than 45..");
//  }
  
 	Serial.print("Misalignment angle: ");
 	Serial.println(theta);
  
 	return theta;
}

/**
 * To make simple comparisons easier for callers, we return 1000 when no
 * wall is seen.
 */
float FirefighterRobot::getFrontWallDistance() {
	float value = getSonarDistance(SONAR_FRONT);

	// Ping library returns 0 when outside of max distance
	if((value == 0) || (value == ROBOT_NO_VALID_DATA)) {
		value = 1000;
	}
	return value;
}

/**
 * Are the sonar close enough to a wall to make for reliable alignment?
 */
boolean FirefighterRobot::isAlignmentPossible(short direction, float maxDistance) {
	boolean bResult = true;

	sonarLocation mySonar[2] = {SONAR_LEFT_R, SONAR_LEFT_F};
	if(direction == ROBOT_RIGHT) {
		mySonar[0] = SONAR_RIGHT_R;
		mySonar[1] = SONAR_RIGHT_F;
	}

	for(int i = 0; i < 2; i++) {
		delay(75); // needed by callers to guarantee clean environment
		float value = getSonarDistance(mySonar[i]);
		if((value == 0) || (value == ROBOT_NO_VALID_DATA) || (value > maxDistance)) {
			bResult = false;
			break;
		}
	}

	return bResult;
}


/**
 * We try to get readings from both sensors, but if only one sensor can get a reading,
 * we'll use that one. Returns ROBOT_NO_VALID_DATA if we don't have a wall, so caller
 * needs to check for that.
 */
float FirefighterRobot::getSideWallDistance(short direction) {
	float distance = 0;
	float thisDistance[2] = {0, 0};
	short numGoodValues = 0;

	sonarLocation mySonar[2] = {SONAR_LEFT_F, SONAR_LEFT_R};
	if(direction == ROBOT_RIGHT) {
		mySonar[0] = SONAR_RIGHT_F;
		mySonar[1] = SONAR_RIGHT_R;
	}

	for(short i = 0; i < 2; i++) {
		// Filter out bad values. NewPing automatically returns 0 for distances past max distance set in
		// constructor
		short numRetries = 0;
		do {
			delay(100);
			thisDistance[i] = sonar[mySonar[i]]->ping_cm();
			numRetries++;
		} while((thisDistance[i] < 0.01) && (numRetries < 5));

		if(thisDistance[i] > 0.01) {
			distance += thisDistance[i];
			numGoodValues++;
		}
	}

	Serial.print("distance1 ");
	Serial.print(thisDistance[0]);
	Serial.print(" distance2 ");
	Serial.println(thisDistance[1]);
	
	if(numGoodValues == 0) {
		return ROBOT_NO_VALID_DATA;
	}
	else {
		distance /= (float)numGoodValues;
	}
	
	return distance;
}

/**
 * This tells us the x-coordinate, in the robot frame, of the point where
 * the IR sensor is contacting the wall. This will depend on the distance
 * from the wall, the angle to the wall, and the relative position of the
 * sensors on the robot.
 */
float FirefighterRobot::getCalculatedWallEnd(short direction) {
	// see what the sensors say first
//	float theta = getMisalignmentAngle(direction);
//	if(theta == ROBOT_NO_VALID_DATA) {
//		Serial.println("Unable to determine wall angle for getIRWallForwardDistance, guessing.");
//		theta = 0;
//	}
//	if(direction == ROBOT_RIGHT) {
//		theta *= -1.0;
//	}

	float d_sonar_to_center = 6.0;
	float x_position_ir = 6.3;  // E
	float d_ir = 2.0;
	float gamma = 47 * DEG_TO_RAD;

	// another sensor call
	float d_sonar_to_wall = getSideWallDistance(direction); // dm
	if((d_sonar_to_wall == ROBOT_NO_VALID_DATA) || (d_sonar_to_wall < 0.01)) {
		Serial.print("Received invalid d_sonar_to_wall value ");
		Serial.println(d_sonar_to_wall);
		Serial.println("Unable to determine wall distance for getIRWallForwardDistance, guessing.");
		d_sonar_to_wall = 23 - d_sonar_to_center;
	}

	// some derived values we'll need
//	float theTanThing = tan(0.5*(PI - (2.0 * gamma) - theta)) / tan(0.5*(PI - theta));
//	float x_base = x_position_ir + ((d_ir + d_sonar_to_wall) * tan((PI/2.0) - gamma));
//
//	float x0 = (d_sonar_to_center + d_sonar_to_wall) * sin(theta);
//	float x1 = x_base * (1 + theTanThing) / (1 - theTanThing);
//
	Serial.print("d_sonar_to_wall: ");
	Serial.println(d_sonar_to_wall);
//	Serial.print("theta: ");
//	Serial.println(theta);
//	Serial.print("x0: ");
//	Serial.println(x0);
//	Serial.print("x1: ");
//	Serial.println(x1);
//
//	float x = x0 + x1;
//	return x;

	// or, do this the crude way, assuming that theta is nearly zero
	float x = (tan((PI/2.0) - gamma) * (d_sonar_to_wall + d_ir)) + x_position_ir;
	Serial.print("distance to end of wall: ");
	Serial.println(x);

	return x;
}

float FirefighterRobot::getFireReading() {
	float reading = 0;
	for(int i = 0; i < 5; i++) {
		reading += analogRead(fireSensorPin);
		delay(10);
	}
	reading /= 5.0;
	return reading;
}

boolean FirefighterRobot::isFire(boolean bNear) {
	float reading = getFireReading();
	if((!bNear) && (reading < fireThresholdReading)) {
		return true;
	}
	if(bNear && (reading < fireThresholdReadingNear)) {
		return true;
	}
	return false;
}

void FirefighterRobot::setFanServo(short degrees) {
	if(degrees < -180) {
		degrees = (degrees + 360) % 360;
	}

	fanServoDegrees = degrees;
	
	int val = map(degrees, -180, 180, 40, 140);
	/* Serial.print("Servo PWM value: ");
	Serial.println(val); */
	fanServo.write(val);
}

int FirefighterRobot::panServoForFire() {
	return panServoForFire(90, -180, false);
}

int FirefighterRobot::panServoForFire(int startDegree, int endDegree, boolean bNear) {
	// only go one way
	if(endDegree > startDegree) {
		float temp = startDegree;
		startDegree = endDegree;
		endDegree = temp;
	}

	int degrees = startDegree;
	boolean fireFound = false;
	
	setFanServo(degrees);
	delay(750);	// let it get all the way there!
	delay(500); 	// and, let fire sensor settle down --VERY IMPORTANT!!
	
	while(degrees > endDegree) {
		delay(60);
		degrees -= 5;
		setFanServo(degrees);
		
		/* for single room, no triple check needed
		if(isFire(false)) {
			// triple check
			delay(100);
			if(isFire(false)) {
				delay(100);
				if(isFire(false)) {
					fireFound = true;
					break;
				}
			}
		} */
		
		if(isFire(false)) {
			fireFound = true;
			break;
		}
	}	
	
	if(fireFound) {		
		Serial.print("Initial fire found at ");
		Serial.println(degrees);
		// Serial.print("Fire reading: ");
		// Serial.println(getFireReading());
	}	
	
	short fireDegrees = degrees;
	
	// scan to find minimum
	if(fireFound) {
		delay(100);
		float reading = getFireReading();
		float minimumReading = reading;
		// Serial.print("Initial reading: ");
		// Serial.println(reading);
		do {
			degrees -= 2;
			setFanServo(degrees);
			delay(15);
			reading = getFireReading();
		
			if(reading < minimumReading) {
				// double check
				delay(30);
				reading = getFireReading();
				if(reading < minimumReading) {
				
					minimumReading = reading;
					fireDegrees = degrees;
					
					// Serial.print("New minimum reading: ");
					// Serial.println(minimumReading);
				}
			}
			
		} while(degrees > endDegree);
	}
	
	if(fireFound) {
		Serial.print("Fire found at ");
		Serial.println(fireDegrees);
		setFanServo(fireDegrees);
		return fireDegrees;
	}	
	
	setFanServo(0);
	return ROBOT_NO_FIRE_FOUND;	// NOT 0 as we may be facing the fire!
}

/* 
int FirefighterRobot::panServoForFire(int startDegree, int endDegree, boolean bNear) {
	int degrees = startDegree;
	
	float delayTime = fabs(startDegree - fanServoDegrees) * 20;
	setFanServo(degrees);
	delay(delayTime);	// let it get all the way there!
	
	if(endDegree < startDegree) {
		while((!isFire(bNear)) && (degrees > endDegree)) {
			delay(30);
			degrees -= 1;
			setFanServo(degrees);
		}	
	}
	else {
		while((!isFire(bNear)) && (degrees < endDegree)) {
			delay(30);
			degrees += 1;
			setFanServo(degrees);
		}	
	}
	
	if(isFire(bNear)) {
		return degrees;
	}
	
	setFanServo(0);
	return ROBOT_NO_FIRE_FOUND;
} */


/**
 *	Robot should be facing the fire when this is called.
 */
void FirefighterRobot::fightFire() {
	resetStallWatcher();

	// drive to candle, re-centering every 25 cm
	while((getFrontWallDistance() > 25) && (!isStalled())) {
		resetStallWatcher();
		resetOdometers();

		odom.markPosition();
		setGoal(25, 0);

		// TODO: remove the delay when sonar no longer requires it
		while((!isStalled()) && (getFrontWallDistance() > 25) && (odom.getDistanceFromMarkedPoint() < 25)) {
			driveTowardGoal();
			odom.update();
			delay(50);
 	 	}	
 	 	stop();
 	 	
 	 	// we've gone as far as we want to using straight travel
 	 	// we can stop slightly further out, though, if needed
 	 	if(getFrontWallDistance() > 35) {
	 	 	int degreesOff = panServoForFire(45, -45, true);
	 	 	if(degreesOff == ROBOT_NO_FIRE_FOUND) {
	 	 		degreesOff = panServoForFire(130, -130, true);
	 	 	}
	 	 	if(degreesOff != ROBOT_NO_FIRE_FOUND) {
		 	 	if(fabs(degreesOff) > 5) {
	 		 		turn(degreesOff * DEG_TO_RAD);
	 		 		delay(250);
	 		 	}
	 		}
	 		else {
	 			Serial.println("Lost our fire!");
	 		}
	 	}
 	}
 	
 	if(isStalled()) {
 		// no recovery
 		resetStallWatcher();
 	}
  
  stop();
  turnFanOn(true);
  
  int degrees = 45;
  setFanServo(degrees); 
  delay(500);
  
  for(int i = 0; i < 5; i++) {
	  while(degrees > -65) {
  		delay(60);
  		degrees -= 5;
  		setFanServo(degrees);
 	 	}
  	while(degrees < 65) {
  		delay(60);
  		degrees += 5;
  		setFanServo(degrees);
  	}
  }
  
  turnFanOn(false);
  setFanServo(0);
}

void FirefighterRobot::resetCalculatedMovePWMs() {
	moveCalculatedPWM = movePWM;
	followWallCalculatedPWM = followWallPWM;
}

/**
 * This also resets the stall watcher, an important piece
 */
void FirefighterRobot::resetOdometers() {
	odom.reset();
	stallWatcher->reset();
}

float FirefighterRobot::recover() {
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
		turn(15 * DEG_TO_RAD);
		angleTurned += 15 * DEG_TO_RAD;
	}
	else {
		turn(-15 * DEG_TO_RAD);
		angleTurned -= 15 * DEG_TO_RAD;
	}
	stop();
	resetStallWatcher();

	return angleTurned;
}
