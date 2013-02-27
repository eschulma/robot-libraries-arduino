#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H
#include <Arduino.h>

class BatteryMonitor {
	private:
		short readPin;
		float fullVoltage;
	public:
		float getRelativeCharge();
		
		BatteryMonitor(short inReadPin, float fullyChargedVoltage);
};
#endif
