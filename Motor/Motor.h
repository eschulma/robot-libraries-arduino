/**
 *	Generic parent class for gear and servo motors. The odometer is a convenience associated
 *	with the motor, usually encoder counts.
 */
#ifndef Motor_h
#define Motor_h
#include <../Encoder/Encoder.h>

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

class Motor {
	private:
		boolean isEncoderPositiveForward; // for clockwise vs. counter-clockwise = forward
		Encoder* encoder;
	protected:
		short enablePin;
		float velocityDeadZone;	// velocities within this normalized range are considered to be zero
		boolean isMotorRunningForward;
		boolean isEncoderSet;

		// children must implement these
		virtual void setMotorDirection(boolean bForward) = 0;
	public:
		virtual void setup() = 0;

		virtual void stop();
		void setVelocityDeadZone(float deadZone) { velocityDeadZone = deadZone; }
		void setEncoder(Encoder* e, boolean isPositiveForward );
		void setNormalizedVelocity(float commandedNormalizedVelocity);
		virtual void setVelocity(int commandedVelocity);

		long getOdometerValue();
		void resetOdometer();

		// DO NOT set encoder to zero!
		Motor(short inEnablePin) { enablePin = inEnablePin; velocityDeadZone = 0.01; isEncoderSet = false; };
};
#endif
