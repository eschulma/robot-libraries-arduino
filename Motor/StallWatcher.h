#ifndef STALL_WATCHER_H
#define STALL_WATCHER_H
#include <Arduino.h>
#include <Encoder.h>

class StallWatcher {
	private:
		long lastTickChange[2];
		long previousTickValue[2];
		long timeout;	// how long do we wait before saying we've stalled
		long startTime;	// how much of beginning startup to do we ignore
		long resetTime;
		long currentSpikeStartTime;
		int currentReadingThreshold;
		long lastStallTime;

		Encoder *e[2];
		short currentSensePin[2];
		
		boolean useEncoders;
		boolean useCurrentSensors;
		
		boolean stallFlag;
	public:
		StallWatcher(int inCurrentReadingThreshold, short pin1, short pin2);
		StallWatcher(Encoder* e1, Encoder* e2);
		
		boolean isStalled();
		void reset();
		void setTimeout(long newVal) { timeout = newVal; };
};
#endif
