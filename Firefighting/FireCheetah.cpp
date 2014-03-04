#include <FireCheetah.h>
#include <Servo.h>
#include <ControllerMotor.h>
#include <Encoder.h>
#include <WheelEncoder.h>
#include <Arduino.h>
// #include <BatteryMonitor.h>
#include <StallWatcher.h>

ControllerMotor robotMotor[2] = { ControllerMotor(ARDUINO_MOTOR_SHIELD_LEFT_ENABLE_PIN, ARDUINO_MOTOR_SHIELD_LEFT_DIRECTION_PIN, ARDUINO_MOTOR_SHIELD_LEFT_BRAKE_PIN),
    ControllerMotor(ARDUINO_MOTOR_SHIELD_RIGHT_ENABLE_PIN, ARDUINO_MOTOR_SHIELD_RIGHT_DIRECTION_PIN, ARDUINO_MOTOR_SHIELD_RIGHT_BRAKE_PIN) };
WheelEncoder robotEncoder[2] = { WheelEncoder(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN, 
									FAULHABER_COUNTS_PER_REVOLUTION
									),
								WheelEncoder(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN,
									FAULHABER_COUNTS_PER_REVOLUTION) };
NewPing robotSonar[5] = { NewPing(SONAR_FORWARD_TRIGGER_PIN, SONAR_FORWARD_ECHO_PIN, 50),
		NewPing(SONAR_LEFT_F_TRIGGER_PIN, SONAR_LEFT_F_ECHO_PIN, 50),
		NewPing(SONAR_LEFT_R_TRIGGER_PIN, SONAR_LEFT_R_ECHO_PIN, 50),
		NewPing(SONAR_RIGHT_F_TRIGGER_PIN, SONAR_RIGHT_F_ECHO_PIN, 50),
		NewPing(SONAR_RIGHT_R_TRIGGER_PIN, SONAR_RIGHT_R_ECHO_PIN, 50) };
																
StallWatcher robotStallWatcher(&robotEncoder[0], &robotEncoder[1]);

FireCheetah::FireCheetah(boolean setupFireSensor) {
	for(int i = 0; i < 2; i++) {
		motor[i] = &robotMotor[i];
		encoder[i] = &robotEncoder[i];
	}
	
	for(int j = 0; j < 5; j++) {
		sonar[j] = &robotSonar[j];
	}	

	bSetupFireSensor = setupFireSensor;
}

void FireCheetah::setup() {
	wallSensorPin[ROBOT_LEFT] = WALL_LEFT_SENSOR_PIN;
	wallSensorPin[ROBOT_RIGHT] = WALL_RIGHT_SENSOR_PIN;
	fanControlPin = FAN_CONTROL_PIN;
	
	for(int i = 0; i < 2; i++) {
		pinMode(wallSensorPin[i], INPUT);
	}
	// pinMode(fireSensorPin, INPUT);
	// digitalWrite(fireSensorPin, HIGH);	// turns on internal pull-up resistor

	// note: this is a blocking call! if fire sensor is not present, we'll hang here
	if(bSetupFireSensor) {
		fireSensor.setup();
	}
		
	pinMode(fanControlPin, OUTPUT);
	turnFanOn(false);
	
	pinMode(SERVO_CONTROL_PIN, OUTPUT);
	fanServo.attach(SERVO_CONTROL_PIN);
	setFanServo(0);
	
	pinMode(START_BUTTON_PIN, INPUT);
	digitalWrite(START_BUTTON_PIN, HIGH);
	pinMode(AUDIO_START_PIN, INPUT);

	wheelDiameter[ROBOT_LEFT] = 6.8; // 7.5; // cm, measured
	wheelDiameter[ROBOT_RIGHT] = 6.7; // 7.4; // wheelDiameter[ROBOT_LEFT] ; //* 1.02;	// cm

	// motors and encoders do their own job of setting pins input/output if setup called
	boolean isMotorForward[2] = {false, true};
	for(int i = 0; i < 2; i++) {
		motor[i]->setup();
		motor[i]->setEncoder(encoder[i], isMotorForward[i]);
	}
			
	rightSideSlowFactor = 1.0;
	leftSideSlowFactor = 1.0; // 0.8;
	turnOneWheelOnly = true;
	turnFudgeFactor = 1.0;

	trackWidth = 17.7; // outer distance, measured
	odom.setup(encoder, isMotorForward, wheelDiameter, (double)trackWidth);

	moveMotorIndex = MOTOR_RIGHT;
	
	stallWatcher = &robotStallWatcher;
		
	maxAllowedPWM = 253; // must be LESS than 254

	// 9V *starting* values -- the drive functions will increase or decrease them
	followWallPWM = 50;
	movePWM = 50;
	turnPWM = 30;

	// these evolve over time
	resetCalculatedMovePWMs();

	// Target velocities, in units of cm/s or rad/s. 1 cm = 0.39 inches, 1 rad ~ 57 degrees
	targetMoveVelocity = 15.0;
	targetFollowVelocity = 20.0;
	targetAngularVelocity = 0.6;
		
	/**
	 * The fire threshold is highly dependent on the ambient UV in the room, primarily from sunlight.
	 * In daytime for my family room 300 works well, but at night it needs to be twice as high. In short:
	 * calibrate on the day of the contest.
	 *
	 * Set this high for single room mode, as we know the fire is there!
	 */
	fireThresholdReading = 50; // 50 C = 122 F
	fireOutReading = 40;
	
	/**
	 * IR values. Tested on white walls, 23 cm distance.
	 * Like other sensor values, calibration is recommended.
	 */
	desiredWallSensorReading[ROBOT_LEFT] = 135; // 150;
	desiredWallSensorReading[ROBOT_RIGHT] = 165; // 410; // 180;

	sideWallLossFactor = 0.7;	// higher number means we declare loss of IR contact at a nearer distance

	// ping the sonar sensors, because it seems the first time they are activated, values are wrong
	getSideWallDistance(ROBOT_LEFT);
	getSideWallDistance(ROBOT_LEFT);
	delay(500);
}

/**
 *	Robot should be facing the fire when this is called.
 *
 *	Sunlight can mess with the fire sensor; if there is a sunny
 *	window in the room, it can may put out a stronger signal than the
 *	candle!
 *
 *	The rules require that we be within 12 inches (30.48 cm) of the candle
 *	before extinguishing it.
 */
void FireCheetah::fightFire(int initDegrees) {
	resetStallWatcher();

	Serial.print("degrees to turn: ");
	Serial.println(initDegrees);

	/**
	 * There is a difference between candles right in front of us and those
	 * far off to the right. For the former, we have to worry about hitting the
	 * left wall (theta too high) and for the latter we have to worry about hitting
	 * the right wall once we get there (theta too low).
	 */
	int firstDegreeOffset = 0;
	int degreeOffset = 10; // good for ones to the right
	if(fabs(initDegrees) < 15) {
		degreeOffset = 0;
	}

	// face the right direction
	setFanServo(0);
	turn((double)(initDegrees + firstDegreeOffset) * DEG_TO_RAD);
	stop();
	delay(500);

	resetStallWatcher();

	// drive to candle, re-centering every 25 cm
	while((getFrontWallDistance() > 25) && (!isStalled())) {
		resetStallWatcher();
		resetOdometers();

		odom.markPosition();
		setGoal(25, 0);

		while((!isStalled()) && (getFrontWallDistance() > 20) && (odom.getDistanceFromMarkedPoint() < 25)) {
			driveTowardGoal();
			odom.update();
			delay(50);
 	 	}
 	 	stop();

 	 	if(isStalled()) {
 	 		// minimal recovery
 	 		resetStallWatcher();
 	 		backUp(2.0);
 	 		resetStallWatcher();
 	 	}

 	 	// we've gone as far as we want to using straight travel
 	 	// we can stop slightly further out, though, if needed
 	 	if(getFrontWallDistance() > 20) {
	 	 	int degreesOff = panServoForFire(45, -45);
	 	 	if(degreesOff == ROBOT_NO_FIRE_FOUND) {
	 	 		degreesOff = panServoForFire(130, -130);
	 	 	}
	 	 	if(degreesOff != ROBOT_NO_FIRE_FOUND) {
 		 		turn((double)(degreesOff + degreeOffset) * DEG_TO_RAD);
 		 		stop();
 		 		delay(250);
	 		}
	 		else {
	 			Serial.println("Lost our fire!");
	 		}
	 	}
 	}

 	if(isStalled()) {
		// minimal recovery
		resetStallWatcher();
		backUp(2.0);
		resetStallWatcher();
 	}

 	stop();

 	// center servo on fire
 	int degrees = panServoForFire(45, -45);
 	if(degrees == ROBOT_NO_FIRE_FOUND) {
 		degrees = panServoForFire(130, -130);
 	}
 	if(degrees != ROBOT_NO_FIRE_FOUND) {
 		setFanServo(degrees);
 	}
 	else {
 		setFanServo(0);
 		// we didn't find the fire???
 		Serial.println("Lost the fire?!?!");
 	}

 	// just a straight blast first
 	turnFanOn(true);

 	// fan for 5 seconds no matter what
 	delay(5000);

 	// fan for another 10 seconds, or until the fire is out
 	float reading;
 	for(int i = 0; i < 100; i++) {
 		delay(100);
 		reading = getFireReading();
 		if(reading <  fireOutReading) {
 			delay(1000);
 			// double check
 			reading = getFireReading();
 			if(reading < fireOutReading) {
 				Serial.print("Fire is out!  ");
 				Serial.println(reading);
 				// give it another few s to make sure the fire is really out!
 				delay(3000);
 				break;
 			}
 			else {
 				// we delayed 1000 for nothing
 				i += 6;
 			}
 		}
 	}
 	Serial.print("final reading: ");
 	Serial.println(reading);
 	turnFanOn(false);

 	// still got a fire?
 	if(isFire()) {
 		delay(10);	// don't bother waiting much
 	}
 	else {
 		delay(5000);
 	}
 	degrees = panServoForFire();
 	if(degrees != ROBOT_NO_FIRE_FOUND) {
 		turn((double)(degrees + degreeOffset) * DEG_TO_RAD);
 		stop();
 		delay(250);

 		turnFanOn(true);

 		// straight blast initially
 		if(degrees != 0) {
 			delay(5000);
 		}

 		// sweep fan while blowing
 		degrees = 45;
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
 	} // if fire found
 	else {
 		Serial.println("No more fire!");
 	}
}

/**
 * To make simple comparisons easier for callers, we return NO_SONAR_FRONT_WALL_SEEN
 * (a high positive number) when no wall is seen.
 */
float FireCheetah::getFrontWallDistance() {
	float value = getSonarDistance(SONAR_FRONT);

	// Ping library returns 0 when outside of max distance
	if((value == 0) || (value == ROBOT_NO_VALID_DATA)) {
		value = NO_SONAR_FRONT_WALL_SEEN;
	}
	return value;
}

/**
 * Are the sonar close enough to a wall to make for reliable alignment?
 */
boolean FireCheetah::isAlignmentPossible(short direction, float maxDistance) {
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
float FireCheetah::getSideWallDistance(short direction) {
	float distance = 0;
	float thisDistance[2] = {0, 0};
	short numGoodValues = 0;

	sonarLocation mySonar[2] = {SONAR_LEFT_F, SONAR_LEFT_R};
	if(direction == ROBOT_RIGHT) {
		mySonar[0] = SONAR_RIGHT_F;
		mySonar[1] = SONAR_RIGHT_R;
	}

	float lowerLimit = 3.5;	// below this, sensor value is likely bad
	long delayTime = 100;

	for(short i = 0; i < 2; i++) {
		// Filter out bad values. NewPing automatically returns 0 for distances past max distance set in
		// constructor
		short numRetries = 0;
		do {
			delay(delayTime);
			thisDistance[i] = sonar[mySonar[i]]->ping_cm();
			numRetries++;

			// double-check values..yes, we need to do this
			if(thisDistance[i] >= lowerLimit) {
				delay(delayTime);
				float distanceConfirm = sonar[mySonar[i]]->ping_cm();
				if(fabs(thisDistance[i] - distanceConfirm) > 2.5) {
					// couldn't duplicate, so ignore it
					thisDistance[i] = 0.0;
				}
			}

		} while((thisDistance[i] < lowerLimit) && (numRetries < 5));

		if(thisDistance[i] > lowerLimit) {
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
 *	Returns the cm by which the left front sensor is farther away than the left
 *	rear sensor. After that basic trigonometry can be used to align the robot with the left wall.
 *
 *	For the right, we reverse front and rear.
 */
float FireCheetah::getMisalignment(short direction) {
	boolean readingFailed = false;

	sonarLocation one = SONAR_LEFT_F;
	sonarLocation two = SONAR_LEFT_R;
	if(direction == ROBOT_RIGHT) {
		one = SONAR_RIGHT_R;
		two = SONAR_RIGHT_F;
	}

	float distance1 = 0;
	float lowerLimit = 3.5;	// below this, sensor value is likely bad
	long delayTime = 100;

	// Filter out bad values. NewPing automatically returns 0 for distances past max distance set in
	// constructor
	short numRetries = 0;
	do {
		delay(delayTime);
		distance1 = sonar[one]->ping_cm();

		// double-check values..yes, we need to do this
		if(distance1 >= lowerLimit) {
			delay(delayTime);
			float distanceConfirm = sonar[one]->ping_cm();
			if(fabs(distance1 - distanceConfirm) > 2.5) {
				// couldn't duplicate, so ignore it
				distance1 = 0.0;
			}
		}

		numRetries++;
	} while((distance1 < lowerLimit) && (numRetries < 5));

	if(distance1 < lowerLimit) {
		readingFailed = true;
	}

	float distance2 = 0;
	if(!readingFailed) {
		delay(delayTime);

		numRetries = 0;
		do {
			delay(100);
			distance2 = sonar[two]->ping_cm();
			numRetries++;

			// double-check values..yes, we need to do this
			if(distance2 >= lowerLimit) {
				delay(delayTime);
				float distanceConfirm = sonar[two]->ping_cm();
				if(fabs(distance2 - distanceConfirm) > 2.5) {
					// couldn't duplicate, so ignore it
					distance2 = 0.0;
				}
			}
		} while((distance2 < lowerLimit) && (numRetries < 5));

		if(distance2 < lowerLimit) {
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
 *	Line up robot parallel to wall. We use the boolean to say if this was
 *	possible or successful. Right now, no safety checks.
 */
boolean FireCheetah::align(short direction) {
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
float FireCheetah::getMisalignmentAngle(short direction) {
	float sensorSpacing = 7.7;

 	float y = getMisalignment(direction);

 	if(y == ROBOT_NO_VALID_DATA) {
 		Serial.println("Unable to get good data for alignment.");
 		return ROBOT_NO_VALID_DATA;
 	}

//  Serial.print("Raw diff: ");
//  Serial.println(y);

 	if(sensorSpacing == 0) {
 		Serial.println("Sensor spacing was zero -- corrupted memory!!!!");
 	}


 	/***
 	 * OLD: if fabs(y) was greater than sensorSpacing, return NaN which leads to
 	 * no action being taken.
 	 */

 	// got too great an angle from diff function
	if(fabs(y) > sensorSpacing) {
		Serial.print("diff function returned a value which would give over 45 degrees: ");
		Serial.println(y);
		return ROBOT_NO_VALID_DATA;
	}

 	float theta = atan2(y, sensorSpacing);
 	double limitAngle = PI/6.0;
 	if(fabs(theta) > limitAngle) {
 		Serial.println("Constraining misalignment angle.");
 		if(y > 0) {
 			theta = limitAngle;
 		}
 		else {
 			theta = -limitAngle;
 		}
 	}

 	// wrap with atan2 for safety
 	theta = atan2(sin(theta), cos(theta));

//  Serial.print("diff: ");
//  Serial.println(y);
//  Serial.print("Raw theta: ");
//  Serial.println(theta);

//  if(fabs(y) > sensorSpacing) {
//  	// more than 45 degrees off
//  	Serial.println("Aligning more than 45..");
//  }

 	Serial.print("Misalignment angle (degrees): ");
 	Serial.println(theta * RAD_TO_DEG);

 	return theta;
}

/**
 * This tells us the x-coordinate, in the robot frame, of the point where
 * the IR sensor is contacting the wall. This will depend on the distance
 * from the wall, the angle to the wall, and the relative position of the
 * sensors on the robot.
 */
float FireCheetah::getCalculatedWallEnd(short direction) {
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

boolean FireCheetah::isWayForwardBlocked() {
	boolean isBlocked = false;

	delay(50);
	if(getFrontWallDistance() < 5) {
		isBlocked = true;
	}
	else if(max(getSideWallDistanceReading(ROBOT_LEFT), getSideWallDistanceReading(ROBOT_RIGHT)) > 550) {
		Serial.println("Way forward blocked!");
		isBlocked = true;
	}

	return isBlocked;
}

/**
 * CM from nearest obstacle for this sonar.
 * This will return ROBOT_NO_VALID_DATA for distances greater than max distance -- be careful.
 */
float FireCheetah::getSonarDistance(sonarLocation loc) {
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
 * Use sonar (the one furthest back) to tell us if we are next to a wall on the side.
 */
boolean FireCheetah::isSideWallPresent(short direction) {
	boolean bWallFound = false;

	sonarLocation loc = SONAR_LEFT_R;
	if(direction == ROBOT_RIGHT) {
		loc = SONAR_RIGHT_R;
	}

	if(getSonarDistance(loc) != ROBOT_NO_VALID_DATA) {
		bWallFound = true;
	}

	return bWallFound;
}
