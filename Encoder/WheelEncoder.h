#ifndef WheelEncoder_h
#define WheelEncoder_h
#include "Encoder.h"
#include <Arduino.h>

#define WW12_COUNTS_PER_REVOLUTION 128
#define FAULHABER_COUNTS_PER_REVOLUTION 485

class WheelEncoder : public Encoder {
	private:
		int countsPerRevolution;
	public:
		WheelEncoder(uint8_t pin1, uint8_t pin2, int inCountsPerRevolution);
		int getCountsPerRevolution() { return countsPerRevolution; };
};
#endif
