#include "StallWatcher.h"

/**
 *	Watch for stalls using the encoders.
 */
StallWatcher::StallWatcher(Encoder* e1, Encoder *e2) {
	e[0] = e1;
	e[1] = e2;

	timeout = 500;
	startTime = 750;
	lastStallTime = 0;
	reset();
	
	useEncoders = true;
	useCurrentSensors = false;
}

/**
 *	Threshold is the trigger value from analog read. Arduino motor shield is calibrated for
 *	3.3V = 2A = 1024 reading. Set the current threshold carefully.
 */
StallWatcher::StallWatcher(int inCurrentReadingThreshold, short pin1, short pin2) {
	timeout = 100;
	lastStallTime = 0;
	
	currentReadingThreshold = inCurrentReadingThreshold;
	
	currentSensePin[0] = pin1;
	currentSensePin[1] = pin2;
	
	for(int i = 0; i < 2; i++) {
		pinMode(currentSensePin[i], INPUT);
	}
	
	useCurrentSensors = true;
	useEncoders = false;
}

/**
 *	Must be called immediately prior to first isStalled call to avoid
 *	false alarms, especially if we are using encoders.
 */
void StallWatcher::reset() {
	long now = millis();
	for(int i = 0; i < 2; i++) {
		lastTickChange[i] = now;
		previousTickValue[i] = 0;
	}
	resetTime = now;
	currentSpikeStartTime = -1;
	stallFlag = false;
}

/**
 *	If using encoders, this needs to be called frequently to avoid false alarms!
 */
boolean StallWatcher::isStalled() {
	long now = millis();

	if(!stallFlag) {
		if(useEncoders) {
			// changed this on 2/17/13 to only trigger if BOTH encoders are stalled
			// (I have no reason for doing this other than code inspection)
			boolean stallFlags[2] = { false, false };

			// get new tick values and see what's going on
			if(now > resetTime + startTime) {
		
				for(int i =0; i < 2; i++) {
					long nowTicks = e[i]->read();
					if(nowTicks != previousTickValue[i]) {
						lastTickChange[i] = now;
						previousTickValue[i] = nowTicks;
						continue;
					}
					else {
						if(now - lastTickChange[i] > timeout) {
							stallFlags[i] = true;
						}
					}		
				} // for
			}	// if start time passed

			if(stallFlags[0] && stallFlags[1]) {
				stallFlag = true;
				lastStallTime = now;
			}
		}
		
		if((!stallFlag) && (useCurrentSensors)) {
			boolean isSaturated = false;
		
			for(int i = 0; i < 2; i++) {
				if(analogRead(currentSensePin[i]) > currentReadingThreshold) {
					isSaturated = true;	
					break;
				}
			}
			
			if(isSaturated) {
				long now = millis();
				if(currentSpikeStartTime != -1) {
					if(now - currentSpikeStartTime > timeout) {
						stallFlag = true;
						lastStallTime = now;
					}
				}
				else if(currentSpikeStartTime == -1) {
					currentSpikeStartTime = now;
				}
			}					
				
			if(!stallFlag) {
				// reset from any earlier transient spikes
				currentSpikeStartTime = -1;			
			}
		}
		
		if(stallFlag) {
			Serial.println("STALLED!");
		}
	} // if stall flag not set
	else {
		/**
		 *  The stall flag is set, but we only hold the stall condition for a second or two,
		 *  to avoid false alarms if a new caller forgets to call reset (but they shouldn't
		 *  count on this -- always call reset first!)
		 */
		// if(now - lastStallTime > resetTime) { -- again, doesn't look right ESK 2/17/13
		if(now - lastStallTime > 3000) {
			reset();
			return false;
		}
	}
	
	return stallFlag;
}
