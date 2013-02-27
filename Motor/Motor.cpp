#include "Motor.h"
#include <math.h>

void Motor::setNormalizedVelocity(float commandedNormalizedVelocity) {
	float commandedVelocity = commandedNormalizedVelocity * 255.0;	
	int v = (int)commandedVelocity;
	setVelocity(v);
}

void Motor::stop() {
	// we need to be able to stop VERY fast
	analogWrite(enablePin, 0);
	setMotorDirection(true);
}

void Motor::setVelocity(int commandedVelocity) {
	// in case someone did something naughty
	// commandedVelocity = constrain(commandedVelocity, -255, 255);
	int velocity = commandedVelocity > 0 ? commandedVelocity : -commandedVelocity ;
	if(velocity < velocityDeadZone) {
		velocity = 0;
	}
	
	// write out new speed
	// this is a performance hit, though nowhere near analogRead
	analogWrite(enablePin, velocity);
	
	// if the motors are changing direction, the caller needs to stop them first.
	// Doing it here causes time delays between motors, very bad!
  
	if(commandedVelocity < 0) {
		setMotorDirection(false);
	}
	else {
		setMotorDirection(true);
	}
}

void Motor::setEncoder(Encoder* e, boolean isPositiveForward ) { 
	encoder = e; 
	isEncoderSet = true; 
	isEncoderPositiveForward = isPositiveForward; 
	resetOdometer();
}

void Motor::resetOdometer() { 
	if(isEncoderSet) { 
		encoder->write(0); 
	} 
}

long Motor::getOdometerValue() {
	long ticks = -1;
	if(isEncoderSet) {
	 	ticks = encoder->read();
		if(!isEncoderPositiveForward) {
			ticks *= -1;
		}
	}
	else {
		Serial.println("Error! Odometer function called before encoder was set.");
	}
	return ticks;
}

