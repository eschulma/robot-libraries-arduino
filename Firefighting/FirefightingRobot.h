/*
 * FirefightingRobot.h
 *
 *  Created on: Apr 19, 2013
 *      Author: ekane
 */

#ifndef FIREFIGHTINGROBOT_H_
#define FIREFIGHTINGROBOT_H_

#include "Arduino.h"
#include <Servo.h>
#include <MLX90620.h>

#define ROBOT_NO_FIRE_FOUND -1000

class FirefightingRobot {
protected:
	Servo fanServo;
	MLX90620 fireSensor;

	// short fireSensorPin;
	short fanControlPin;
	short fireThresholdReading;
	short fireOutReading;

	int panServoForFire(int startDegree, int endDegree);
	float getFireReading();

	// children must override these
	virtual void setup() = 0;
public:
	void setFanServo(short degrees);	// 0 is pointing forward
	void turnFanOn(boolean on);
	int panServoForFire();
	boolean isFire();

	FirefightingRobot();
};

#endif /* FIREFIGHTINGROBOT_H_ */
