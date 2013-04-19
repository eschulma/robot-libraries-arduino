#include <FireCheetah.h>
#include <ControllerMotor.h>
#include <Encoder.h>
#include <WheelEncoder.h>
#include <Arduino.h>
// #include <BatteryMonitor.h>
#include <StallWatcher.h>
#include <Servo.h>

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

FireCheetah::FireCheetah() {
	for(int i = 0; i < 2; i++) {
		motor[i] = &robotMotor[i];
		encoder[i] = &robotEncoder[i];
	}
	
	for(int j = 0; j < 5; j++) {
		sonar[j] = &robotSonar[j];
	}	
}

void FireCheetah::setup() {
	wallSensorPin[ROBOT_LEFT] = WALL_LEFT_SENSOR_PIN;
	wallSensorPin[ROBOT_RIGHT] = WALL_RIGHT_SENSOR_PIN;
	fireSensorPin = FIRE_SENSOR_PIN;
	fanControlPin = FAN_CONTROL_PIN;
	
	for(int i = 0; i < 2; i++) {
		pinMode(wallSensorPin[i], INPUT);
	}
	pinMode(fireSensorPin, INPUT);
	// digitalWrite(fireSensorPin, HIGH);	// turns on internal pull-up resistor
		
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
	fireThresholdReading = 600;
	fireOutReading = 1000;
	
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

 	float thisFireReadingThreshold = getFireReading() + 300.0;
 	thisFireReadingThreshold = max(thisFireReadingThreshold, fireOutReading);

 	// just a straight blast first
 	turnFanOn(true);

 	// fan for 5 seconds no matter what
 	delay(5000);

 	// fan for another 10 seconds, or until the fire is out
 	float reading;
 	for(int i = 0; i < 100; i++) {
 		delay(100);
 		reading = getFireReading();
 		if(reading >  thisFireReadingThreshold) {
 			delay(1000);
 			// double check
 			reading = getFireReading();
 			if(reading > thisFireReadingThreshold) {
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
