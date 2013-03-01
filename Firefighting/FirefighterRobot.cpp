#include "FirefighterRobot.h"


FirefighterRobot::FirefighterRobot() {
	// set everything to blank or default values (but not motor or encoder!!)
	trackWidth = 0;
	bangBangDelta = 20;

	for(int i = 0; i < 2; i++) {
		desiredWallSensorReading[i] = 340;
	}

	// consider using a potentiometer for setting these
	moveSpeed = 0.0f;
	turnSpeed = 0.0f;
	followWallSpeed = 0.0f;

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
 * We can operate in the existing world frame, or reset the world frame to be
 * the robot frame. For multiple maneuvers the robot frame is safer,
 * you will at least get the right move** relative to the robot's current location**
 * and that may be the most important thing. However, any Odometer goal or mark points
 * will be invalidated; to keep them, set resetWorldFrameToRobot false.
 *
 * Caller is responsible for calling stop() after this function, if that is the
 * desired behavior, otherwise the bot will keep going.
 *
 * Return true for success, false for failure.
 */
boolean FirefighterRobot::move(double forward, double left, boolean resetWorldFrameToRobot) {
	if(!resetWorldFrameToRobot) {
		// must transform from robot frame to world frame
		odom.transformRobotPointToOdomPoint(&forward, &left);
	}
	else {
		odom.setCurrentPosition(0, 0, 0);
	}
	// now forward and left are really x and y in the world frame
	return goToGoal(forward, left);
}


/**
 * Move the robot to the specified goal point in world coordinates.
 * We don't care about the final heading.
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

	float baseSpeed = moveSpeed;	// units from 0 to 254, input for drive command
	float targetVelocity = targetMoveVelocity;	// cm per sec
	int minSpeed = 1;
	int maxSpeed = 254;

	odom.setGoalPosition(goalX, goalY);
	odom.update();

	Serial.print("Goal heading (degrees): ");
	Serial.println(odom.calculateGoalHeading() * RAD_TO_DEG);
	Serial.print("Current heading (degrees): ");
	Serial.println(odom.getHeading() * RAD_TO_DEG);
//	Serial.print("Heading error (degrees): ");
//	Serial.println(odom.getHeadingError() * RAD_TO_DEG);

	double maxDistanceError = 1;
	double minDistance = 10000;
	double distanceToGoal;
	double distanceNoiseMargin = 3;
	double headingError = 0;
	double velocityError = 0;

	boolean wasSlowDownPerformed = false;
	double slowDownFactor = 0.5;
	double slowDownDistance = 1;

	// PID values. One set for heading error and one set for total velocity
	double kP = 16;
	double kP_velocity = 0.01;

	stallWatcher->reset();

	int dlRaw, drRaw;
	long nLoops = 0;

	do {
		headingError = odom.getHeadingError();	// this will be between 1 and -1

		dlRaw = (int)ceil(baseSpeed * (1 + (kP * headingError)));
		drRaw = (int)ceil(baseSpeed * (1 - (kP * headingError)));

		// don't allow drive to go from positive to negative...bad for motors; and 254 is max
		drive(constrain(dlRaw, minSpeed, maxSpeed), constrain(drRaw, minSpeed, maxSpeed));
		odom.update();

		// figure out the right drive voltage to use for our desired speed
		// first two iterations ignore velocity, it will be undefined
		if(nLoops > 5) {
			velocityError = fabs(odom.getLinearVelocity()) - fabs(targetVelocity); // positive if we are too fast, negative for slow
		}
//		Serial.print("Velocity error: ");
//		Serial.println(velocityError);

		baseSpeed *= (1 - (kP_velocity * velocityError));
		baseSpeed = constrain(baseSpeed, minSpeed, maxSpeed); // mainly needed for debugging

		distanceToGoal = odom.getDistanceToGoal();
//		Serial.print("Distance to goal:  ");
//		Serial.println(distanceToGoal);

		if(!wasSlowDownPerformed) {
			if(distanceToGoal < slowDownDistance) {
				targetVelocity *= slowDownFactor;
				wasSlowDownPerformed = true;
				Serial.println("Slowing down.");
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

		// sampling rate matters quite a bit here
		delay(10);

		// never give up before we've reached full power
		if((stallWatcher->isStalled()) && (fabs(baseSpeed) < 250)) {
			stallWatcher->reset();
		}

		nLoops++;
	} while((!stallWatcher->isStalled()) && (distanceToGoal > maxDistanceError));

	Serial.print("Distance to goal: ");
	Serial.println(distanceToGoal);

	if(distanceToGoal <= maxDistanceError) {
		bSuccess = true;
	}

	return bSuccess;
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

	float baseSpeed = turnSpeed;	// units from 0 to 254, input for drive command
	float targetVelocity = targetAngularVelocity;	// rad per sec

	// for positive angle turn, right wheel is positive velocity, left is negative
	int minLSpeed = -254;
	int maxLSpeed = 0;
	int minRSpeed = 0;
	int maxRSpeed = 254;

	// turn in the specified direction
	if(headingChange < 0) {
		baseSpeed *= -1;
		targetVelocity *= -1;

		minLSpeed = 0;
		maxLSpeed = 254;
		minRSpeed = -254;
		maxRSpeed = 0;
	}

	double maxAngleError = 0.02;
	double minAngularDistance = 10000;
	double radsFromGoalAngle = 0.0;
	double angularNoiseMargin = 0.02;
	double velocityError = 0.0;

	boolean wasSlowDownPerformed = false;
	double slowDownFactor = 0.5;
	double slowDownAngularDistance = 0.04;

	// PID values
	double kP_accel = 2;

	stallWatcher->reset();

	odom.setTargetHeading(goalHeading);
	odom.reset();
	odom.update();
	delay(10);

	float dr = baseSpeed;
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
		dr = constrain(dr, minRSpeed, maxRSpeed);
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
				Serial.println("Slowing down.");
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

	Serial.print("Radians to goal: ");
	Serial.println(radsFromGoalAngle);

	if(radsFromGoalAngle <= maxAngleError) {
		bSuccess = true;
	}

	return bSuccess;
}

/**
 *	Move the robot a short distance straight forward (or back), based on the encoder value
 *	for one motor. There is no error correction if the robot drifts to the side.
 *	Positive moves forward.
 *
 *	The caller is responsible for calling stop() after the function, otherwise the bot
 *	will keep moving.
 *
 *	NOT RECOMMENDED IF YOU NEED ACCURACY. Use goToGoal instead.
 **/
void FirefighterRobot::move(float distance) {
	// convert distance to encoder ticks
	// distance = ticks * wheel circumference * (1 / ticks per revolution), or
	// ticks = distance * ticks per revolution / wheel circumference
	long ticks = fabs(floor( (distance * encoder[ moveMotorIndex ]->getCountsPerRevolution()) / 
		(wheelDiameter[ moveMotorIndex ] * M_PI) ));

	// TODO: safety cap on while loops and make sure we don't bang into anything either
	float baseSpeed = moveSpeed;
	if(distance < 0) {
		baseSpeed *= -1;
	}
	
	resetOdometers();
	stallWatcher->reset();

	do {
		// no PID
		drive(baseSpeed, baseSpeed);
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
void FirefighterRobot::initDesiredWallSensorReadings(short direction) {
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
 *	The key to everything. 
 */
void FirefighterRobot::followWall(short direction, int optimalWallSensorReading) {
	static float followWallCalculatedSpeed = followWallSpeed;

	int driveSpeed[2];

	// PID values. One set for heading error and one set for total velocity
	float kP = 0.02;
	float kP_velocity = 0.01;

	short oppositeDirection = !direction;	// this works for right and left only, note
	
	// Standard analog read here is slow!!!
	// note that IR reading is higher for closer objects
	int wallDistance = (int)getSideWallDistanceReading(direction);
	int headingError = wallDistance - desiredWallSensorReading[direction];

	driveSpeed[direction] = followWallCalculatedSpeed * (1 + (kP * headingError));
	driveSpeed[oppositeDirection] = followWallCalculatedSpeed * (1 - (kP * headingError));

	// we only go forward here; error is positive if we are too fast
	odom.update();
	float velocityError = odom.getLinearVelocity() - targetFollowVelocity;
	followWallCalculatedSpeed *= (1 - (kP_velocity * velocityError));
	
//	Serial.print("Velocity error: ");
//	Serial.println(velocityError);

	// let's go
	drive(constrain(driveSpeed[MOTOR_LEFT], 0, 254), constrain(driveSpeed[MOTOR_RIGHT], 0, 254));
}

void FirefighterRobot::followWallRear(short direction, int optimalWallSensorReading) {
// TODO: copy other follow wall, mostly
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

float FirefighterRobot::getSonarWallDistance(sonarLocation loc) {
	// Serial.print("Getting distance for sonar wall ");
	// Serial.println(loc);

	long microsec = sonar[loc]->timing();
	//Serial.print("Microsec " );
	//Serial.println(microsec);
	
	float wallDistance = sonar[loc]->convert(microsec, Ultrasonic::CM);
	//Serial.print("Wall distance ");
	//Serial.println(wallDistance);
	
	return wallDistance;
}

/**
 *	Returns the cm by which the left front sensor is farther away than the left
 *	rear sensor.
 *	After that basic trigonometry can be used to align the robot with the left wall.
 */
float FirefighterRobot::getMisalignment(short direction) {
	float maxDistanceValue = 100;
	boolean readingFailed = false;

	sonarLocation front = SONAR_LEFT_F;
	sonarLocation rear = SONAR_LEFT_R;
	if(direction == ROBOT_RIGHT) {
		front = SONAR_RIGHT_R;
		rear = SONAR_RIGHT_F;
	}

	long reading1 = 0;
	long reading2 = 0;
	
	reading1 = sonar[front]->timing();
	float distance1 = sonar[front]->convert(reading1, Ultrasonic::CM);
	
	// filter out bad values
	short numRetries = 0;
	while((distance1 > maxDistanceValue) && (numRetries < 5)) {
		delay(100);
		reading1 = sonar[front]->timing();
		distance1 = sonar[front]->convert(reading1, Ultrasonic::CM);
		numRetries++;
	}
	if(distance1 > maxDistanceValue) {
		readingFailed = true;
	}
	
	float distance2 = 0;
	if(!readingFailed) {
		delay(100);
		
		reading2 = sonar[rear]->timing();	
		distance2 = sonar[rear]->convert(reading2, Ultrasonic::CM);
	
		numRetries = 0;
		while((distance2 > maxDistanceValue) && (numRetries < 5)) {
			delay(100);
			reading2 = sonar[rear]->timing();
			distance2 = sonar[rear]->convert(reading1, Ultrasonic::CM);
			numRetries++;
		}	
		if(distance2 > maxDistanceValue) {
			readingFailed = true;
		}
	}
	
	Serial.print("Front distance ");
	Serial.print(distance1);
	Serial.print(" Rear distance ");
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
  Serial.print("Aligning robot, side ");
  Serial.println(direction);
 
	float sensorSpacing = 7.7;

 	float y = getMisalignment(direction);
 	
 	if((y == ROBOT_NO_VALID_DATA) || (y == 0)) {
 		Serial.println("Unable to get good data for alignment.");
 		return false;
 	}
 	
  Serial.print("Raw diff: ");
  Serial.println(y);

	// got back garbage from diff function
 	if(fabs(y) > 160) {
 		return false;
  }
  
  float theta = atan2(y, sensorSpacing);
  
  Serial.print("diff: ");
  Serial.println(y);
  Serial.print("Raw theta: ");
  Serial.println(theta);
      
  if(fabs(y) > sensorSpacing) {
  	// more than 45 degrees off
  	Serial.println("Aligning more than 45..");
  }    

  theta *= (180.0 / M_PI);
  
  // my fudge
  theta += 7;
  
  if(fabs(theta) > 1) {
	  turn(theta);
	}
  stop();
  
  Serial.print("Aligned robot, side ");
  Serial.println(direction);
  Serial.print("Angle: ");
  Serial.println(theta);
  
  return true;
}

float FirefighterRobot::getSideWallDistance(short direction) {
	float distance = 0;

	sonarLocation sonar[2] = { SONAR_LEFT_F, SONAR_LEFT_R };
	if(direction == ROBOT_RIGHT) {
		sonar[0] = SONAR_RIGHT_F;
		sonar[1] = SONAR_RIGHT_R;
	}
	
	
	for(int i = 0; i < 2; i++) {
		distance += getSonarWallDistance(sonar[i]);
	}
	
	distance /= 2.0;
	return distance;
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

	// drive to candle, re-centering every 30 cm
	while((getFrontWallDistance() > 25) && (!isStalled())) {
		resetStallWatcher();
		resetOdometers();

		odom.setGoalPosition(0, 0);
		odom.update();
		while((!isStalled()) && (getFrontWallDistance() > 25) && (odom.getDistanceToGoal() < 25)) {
			drive(moveSpeed, moveSpeed);
			delay(75);
			odom.update();
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
	 		 		turn(degreesOff);
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

void FirefighterRobot::recover() {
	stop();
	delay(500);
	resetStallWatcher();
	move(-10);
	resetStallWatcher();
}

