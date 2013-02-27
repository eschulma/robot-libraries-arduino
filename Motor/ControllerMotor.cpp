#include "ControllerMotor.h"

ControllerMotor::ControllerMotor(short pwmEnablePin, short inDirectionPin, short inBrakePin) : Motor(pwmEnablePin) {
	directionPin = inDirectionPin;
	brakePin = inBrakePin;
	isMotorRunningForward = true;
	isBrakeSet = false;
}

void ControllerMotor::setup() {
	pinMode(enablePin, OUTPUT);	// PWM
	pinMode(directionPin, OUTPUT);
	if(brakePin != -1) {
		pinMode(brakePin, OUTPUT);
	}
	stop();
	setMotorDirection(true);
	
	// be ready
	releaseBrake();
}

void ControllerMotor::setMotorDirection(boolean bForward) {
	if(bForward) {
	  digitalWrite(directionPin, HIGH); 
	  isMotorRunningForward = true;
	}
	else {
	  digitalWrite(directionPin, LOW);
	  isMotorRunningForward = false;    
	}  
}

void ControllerMotor::stop() {
	Motor::stop();
	brake();
}

void ControllerMotor::setVelocity(int commandedVelocity) {
	if(isBrakeSet) {
		if(fabs(commandedVelocity) != 0) {
			releaseBrake();
		}
	}
	Motor::setVelocity(commandedVelocity);
}

void ControllerMotor::brake() {
	if(brakePin != -1) {
		digitalWrite(brakePin, HIGH);
		isBrakeSet = true;
		// Serial.println("Brake applied.");
	}
}

void ControllerMotor::releaseBrake() {
	if(brakePin != -1) {
		digitalWrite(brakePin, LOW);
		isBrakeSet = false;
		// Serial.println("Brake released.");
	}
}

