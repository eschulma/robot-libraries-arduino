#ifndef L293GearMotor_h
#define L293GearMotor_h
#include <Motor.h>

class L293GearMotor : public Motor {
	private:
		short input1Pin;
		short input2Pin;

	void setMotorDirection(boolean bForward) {
		if(bForward) {
		  digitalWrite(input1Pin, LOW);
		  digitalWrite(input2Pin, HIGH); 
		  isMotorRunningForward = true;
		}
		else {
		  digitalWrite(input1Pin, HIGH);
		  digitalWrite(input2Pin, LOW);
		  isMotorRunningForward = false;    
		}  
	}

	public:
	L293GearMotor(short pwmEnablePin, short inInput1Pin, short inInput2Pin) {
		enablePin = pwmEnablePin;
		input1Pin = inInput1Pin;
		input2Pin = inInput2Pin;
		setVelocityDeadZone(0.01);
		isMotorRunningForward = true;
	}

	void setup() {
		pinMode(enablePin, OUTPUT);	// PWM
		pinMode(input1Pin, OUTPUT);
		pinMode(input2Pin, OUTPUT);
		stop();
	}
};
#endif

