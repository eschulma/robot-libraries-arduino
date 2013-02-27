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
									FAULHABER_COUNTS_PER_REVOLUTION // * 1.2
									),
								WheelEncoder(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN,
									FAULHABER_COUNTS_PER_REVOLUTION) };
Ultrasonic robotSonar[5] = { Ultrasonic(SONAR_FORWARD_TRIGGER_PIN, SONAR_FORWARD_ECHO_PIN), 
							Ultrasonic(SONAR_LEFT_F_TRIGGER_PIN, SONAR_LEFT_F_ECHO_PIN),
							Ultrasonic(SONAR_LEFT_R_TRIGGER_PIN, SONAR_LEFT_R_ECHO_PIN),
							Ultrasonic(SONAR_RIGHT_F_TRIGGER_PIN, SONAR_RIGHT_F_ECHO_PIN),
							Ultrasonic(SONAR_RIGHT_R_TRIGGER_PIN, SONAR_RIGHT_R_ECHO_PIN) };		
																
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
		pinMode(wallRearSensorPin[i], INPUT);
	}
	pinMode(fireSensorPin, INPUT);
	digitalWrite(fireSensorPin, HIGH);	// turns on internal pull-up resistor
		
	pinMode(fanControlPin, OUTPUT);
	turnFanOn(false);
	
	pinMode(SERVO_CONTROL_PIN, OUTPUT);
	fanServo.attach(SERVO_CONTROL_PIN);
	setFanServo(0);
	
	pinMode(START_BUTTON_PIN, INPUT);
	digitalWrite(START_BUTTON_PIN, LOW);

	wheelDiameter[ROBOT_LEFT] = 5.0;	// cm 5.7 by spec, 5.5 measured
	wheelDiameter[ROBOT_RIGHT] = wheelDiameter[ROBOT_LEFT] ; //* 1.02;	// cm

	// motors and encoders do their own job of setting pins input/output if setup called
	boolean isMotorForward[2] = {true, false};
	for(int i = 0; i < 2; i++) {
		motor[i]->setup();
		motor[i]->setEncoder(encoder[i], isMotorForward[i]);
	}
			
	rightSideSlowFactor = 1.0;
	leftSideSlowFactor = 1.0; // 0.8;
	turnOneWheelOnly = true;
	turnFudgeFactor = 1.0;

	trackWidth = 17.7; // outer distance, measured. 15.1 originally
	odom.setup(encoder, isMotorForward, wheelDiameter, (double)trackWidth);

	moveMotorIndex = MOTOR_RIGHT;
	
	stallWatcher = &robotStallWatcher;
		
	// BatteryMonitor monitor(BATTERY_MONITOR_PIN, 3.6);
	// batteryChargeLevel = monitor.getRelativeCharge();	
	// a hack, but:
	// if(batteryChargeLevel > 1.0) {
		batteryChargeLevel = 1.0;
	// }

	// Tunable parameters -- start slow! Maximum speed is 254
	//
	/* 12V values (at 12V and 30, won't move (too low)
	followWallSpeed = floor(70.0 / batteryChargeLevel);
	bangBangDelta = floor(30.0 / batteryChargeLevel);
	moveSpeed = floor(40.0 / batteryChargeLevel);
	turnSpeed = floor(40.0 / batteryChargeLevel); */

	// 9V values
	followWallSpeed = floor(93.3 / batteryChargeLevel);
	bangBangDelta = floor(40.0 / batteryChargeLevel);
	moveSpeed = 65; // 90; // floor(5653.3 / batteryChargeLevel);
	turnSpeed = 30;

	// Target velocities, in units of cm/s or rad/s. 1 cm = 0.39 inches, 1 rad ~ 57 degrees
	targetMoveVelocity = 15.0;
	targetAngularVelocity = 0.6;
		
	for(int i = 0; i < 2; i++) {
		desiredWallSensorReading[i] = 120;
	}
	sideWallLossFactor = 0.8;	// higher number means we lose it at a nearer distance
	// set this high for single room, as we know the fire is there!
	fireThresholdReading = 75; // but this is a small room, and reflections off walls a problem.
	fireThresholdReadingNear = 50;
	
	frontWallFoundDistance = 35;	// this relates to hallway width and stop distance
	rearWallFoundDistance = 35;		// this relates to hallway width and stop distance
}
