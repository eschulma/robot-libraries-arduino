/*
 * FirefightingRobot.cpp
 *
 *  Created on: Apr 19, 2013
 *      Author: ekane
 */
#include "Arduino.h"
#include "FirefightingRobot.h"

FirefightingRobot::FirefightingRobot() {
	// TODO Auto-generated constructor stub
}

int FirefightingRobot::panServoForFire() {
	return panServoForFire(90, -180);
}

int FirefightingRobot::panServoForFire(int startDegree, int endDegree) {
	// only go one way
	if(endDegree > startDegree) {
		int temp = startDegree;
		startDegree = endDegree;
		endDegree = temp;
	}

	int degrees = startDegree;
	boolean fireFound = false;

	setFanServo(degrees);
	delay(750);	// let it get all the way there!
	delay(500); 	// and, let fire sensor settle down --VERY IMPORTANT!!

	// scan quickly until a fire is detected, then slow down
	while(degrees > endDegree) {
		delay(60);
		degrees -= 5;
		setFanServo(degrees);

		if(isFire()) {
			// double-checking does not seem to work
			fireFound = true;
			break;
		}
	}

	if(fireFound) {
		Serial.print("Initial fire found at ");
		Serial.println(degrees);
		Serial.print("Fire reading: ");
		Serial.println(getFireReading());
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

boolean FirefightingRobot::isFire() {
	float reading = getFireReading();
	if(reading > fireThresholdReading) {
		return true;
	}
	return false;
}

//boolean FirefighterRobot::isFireOut() {
//	float reading = getFireReading();
//	float threshold = fireThresholdReading + 100.0;
//	if(reading > threshold) {
//		return true;
//	}
//	return false;
//}

void FirefightingRobot::setFanServo(short degrees) {
	if(degrees < -180) {
		degrees = (degrees + 360) % 360;
	}

	int val = map(degrees, -180, 180, 40, 140);
	/* Serial.print("Servo PWM value: ");
	Serial.println(val); */
	fanServo.write(val);
}

float FirefightingRobot::getFireReading() {
	float reading = 0;
	for(int i = 0; i < 5; i++) {
		reading += fireSensor.getMaxTemperature();
		delay(10);
	}
	reading /= 5.0;
	return reading;
}

void FirefightingRobot::turnFanOn(boolean on) {
	if(on) {
		digitalWrite(fanControlPin, HIGH);
	}
	else {
		digitalWrite(fanControlPin, LOW);
	}
}
