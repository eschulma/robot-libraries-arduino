#include "WheelEncoder.h"

WheelEncoder::WheelEncoder(uint8_t pin1, uint8_t pin2, int inCountsPerRevolution) : Encoder(pin1, pin2) {
	countsPerRevolution = inCountsPerRevolution;
}
