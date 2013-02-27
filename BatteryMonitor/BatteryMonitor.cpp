#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(short inReadPin, float fullyChargedVoltage) { 
	readPin = inReadPin;
	fullVoltage = fullyChargedVoltage;
	pinMode(inReadPin, INPUT);
}

/**
 *	We assume default analog reference value of 5V. This returns 1 for a fully
 *	charged battery.
 */
float BatteryMonitor::getRelativeCharge() {
	int raw = analogRead(readPin);
 	float val = raw * (5.0 / 1024.0);
 	float percentage = val/fullVoltage;
 	return percentage;
}
