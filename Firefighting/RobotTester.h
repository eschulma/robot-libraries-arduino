#ifndef RobotTester_h
#define RobotTester_h

#include <Arduino.h>
#include <Motor.h>
#include <ControllerMotor.h>
#include <FirefighterRobot.h>
#include <FireCheetah.h>

class RobotTester {
	private:
		FirefighterRobot* robot;

	public:
		RobotTester(FirefighterRobot *inRobot) { robot = inRobot; };
	
void testWallLoss() {
  for(int i = 0; i < 400; i++) {
    boolean lost = robot->isSideWallLost(ROBOT_RIGHT);
    Serial.println(lost);
    delay(250);
  }
}

void testWallFollowing() {
  long start = millis();
  long numFollows = 0;
  int wallSide = ROBOT_LEFT;

  robot->initDesiredWallSensorReadings(wallSide);
  robot->resetOdometers();
  robot->resetStallWatcher();
  robot->followWallSpeed = 70;
  
  while(!robot->isSideWallLost(wallSide)) {
    robot->followWall(wallSide);
    numFollows++;
  }    
  float lossDistanceReading = robot->getSideWallDistanceReading(wallSide);
  robot->stop();
  long end = millis();
  Serial.print("Lost wall at wall distance ");
  Serial.println(lossDistanceReading);
  Serial.print("numFollows: ");
  Serial.println(numFollows);
  if((end - start) > 0) {
    Serial.print("numFollows frequency: ");
    Serial.println((numFollows * 1000.0)/(end - start));
  }
}

void testFireSensors() {
	float reading;
	while(1) {	
		reading = robot->getFireReading();
		Serial.println(reading);
		delay(1000);
	}
}

void testEncoders() {
	// robot->drive(100, 100);

	long leftTicks = 0;
	long rightTicks = 0;
	robot->resetOdometers();

	for(int i = 0; i < 4*60; i++) {
		leftTicks = robot->getOdometerValue(ROBOT_LEFT);
		rightTicks = robot->getOdometerValue(ROBOT_RIGHT);

		Serial.print("left ticks: ");
		Serial.print(leftTicks);
		Serial.print("     right ticks: ");
		Serial.println(rightTicks);
		delay(250);
	}
	robot->stop();
}

/**
 * Do we have the right value in here for numbers of ticks
 * given by a full revolution of the wheel?
 */
void testEncoderClicksPerRev() {
	short index = MOTOR_LEFT;
	int stopTicks = robot->encoder[index]->getCountsPerRevolution();
	Serial.print("stop ticks: ");
	Serial.println(stopTicks);

	long ticks[2] = {0, 0};
	robot->resetOdometers();

	Serial.println("Starting to drive...");

	robot->drive(20, 20);
	while(ticks[index] < stopTicks) {
		ticks[0] = robot->getOdometerValue(ROBOT_LEFT);
		ticks[1] = robot->getOdometerValue(ROBOT_RIGHT);
	}
	robot->stop();

	Serial.print("left ticks: ");
	Serial.print(ticks[0]);
	Serial.print("     right ticks: ");
	Serial.println(ticks[1]);

	delay(500);
	ticks[0] = robot->getOdometerValue(ROBOT_LEFT);
	ticks[1] = robot->getOdometerValue(ROBOT_RIGHT);
	Serial.println("");
	Serial.println("1/2s after stop --");
	Serial.print("left ticks: ");
	Serial.print(ticks[0]);
	Serial.print("     right ticks: ");
	Serial.println(ticks[1]);

	delay(2000);
	ticks[0] = robot->getOdometerValue(ROBOT_LEFT);
	ticks[1] = robot->getOdometerValue(ROBOT_RIGHT);
	Serial.println("");
	Serial.println("More than 2s after stop --");
	Serial.print("left ticks: ");
	Serial.print(ticks[0]);
	Serial.print("     right ticks: ");
	Serial.println(ticks[1]);
}

void testCircling() {
	robot->drive(-100, 100);
}

void testTurning() {
  // robot->move(50);
  // delay(2000);
  robot->turn(-90);
  delay(2000);
  robot->turn(90);
}

void testTurnToHeading() {
  long leftTicks, rightTicks;
  robot->resetOdometers();

  robot->turnToHeading(PI/2.0);
  robot->stop();
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  Serial.println("Ticks right after stop: ");
  Serial.print("left ticks: ");
  Serial.print(leftTicks);
  Serial.print("     right ticks: ");
  Serial.println(rightTicks);

  delay(2000);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  Serial.println("Ticks after delay: ");
  Serial.print("left ticks: ");
  Serial.print(leftTicks);
  Serial.print("     right ticks: ");
  Serial.println(rightTicks);

  robot->turnToHeading(-PI/2.0);
  robot->stop();

  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  Serial.println("Ticks right after stop: ");
  Serial.print("left ticks: ");
  Serial.print(leftTicks);
  Serial.print("     right ticks: ");
  Serial.println(rightTicks);

  delay(2000);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  Serial.println("Ticks after delay: ");
  Serial.print("left ticks: ");
  Serial.print(leftTicks);
  Serial.print("     right ticks: ");
  Serial.println(rightTicks);

  robot->stop();
}

void testMoving() {
	robot->resetOdometers();
	robot->move(15);
	robot->stop();
}

void testMovingToGoal() {
  robot->resetOdometers();
  // robot->moveStopDistance = 0;
  // robot->moveSpeed = 40;
  robot->goToGoal(10, 0);

  long leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  long rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  
  Serial.print("left ticks: ");
  Serial.print(leftTicks);
  Serial.print("     right ticks: ");
  Serial.println(rightTicks);

  robot->stop();
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  Serial.println("Ticks right after stop: ");
  Serial.print("left ticks: ");
  Serial.print(leftTicks);
  Serial.print("     right ticks: ");
  Serial.println(rightTicks);
  
  delay(2000);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  Serial.println("Ticks after delay: ");
  Serial.print("left ticks: ");
  Serial.print(leftTicks);
  Serial.print("     right ticks: ");
  Serial.println(rightTicks);
}

void testFrontSonar() {
	while(1) {
		delay(500);
		Serial.println(robot->getFrontWallDistance());
	}
}

void testIndividualSonar() {
	while(1) {
		delay(500);
		long timing = robot->sonar[SONAR_LEFT_F]->timing();
		float distance1 = robot->sonar[SONAR_LEFT_F]->convert(timing, Ultrasonic::CM);
		Serial.println(distance1);
	}
}

void testIR() {
  while(1) {
    delay(250);
    Serial.print(analogRead(WALL_LEFT_SENSOR_PIN));
    Serial.print(" ");
    Serial.println(analogRead(WALL_RIGHT_SENSOR_PIN));
  }
}

void testFan() {
	robot->turnFanOn(true);
	delay(5000);
	robot->turnFanOn(false);
}

void testAlignmentReadings() {
  while(1) {
    delay(500);
    float diff = robot->getMisalignment(ROBOT_LEFT);
  	  Serial.print("distance diff: ");
  	  Serial.println(diff);
    if(diff != 0) {
	    float theta = atan2(diff, 7.7) * (180.0 / M_PI);
  	  Serial.print("angle: ");
  	  Serial.println(theta);
  	 }
  }
}

void testAligning() {
  robot->align(ROBOT_LEFT);
}

/**
 *	Robot will not drive straight for a while after you do this!
 */
void testStallCutoff() {	
	int maxCurrent = 0;
	int leftCurrent;
	int rightCurrent;
	
	robot->resetStallWatcher();
	robot->drive(robot->getMoveSpeed(), robot->getMoveSpeed());	
	for(int i = 0; ((i < 10000) && (!robot->isStalled())); i++) {
		leftCurrent = analogRead(ARDUINO_MOTOR_SHIELD_LEFT_CURRENT_SENSE_PIN);
		rightCurrent = analogRead(ARDUINO_MOTOR_SHIELD_RIGHT_CURRENT_SENSE_PIN);
		if((leftCurrent > 100) || (rightCurrent > 100)) {
			Serial.print(leftCurrent);
			Serial.print(" ");
			Serial.println(rightCurrent);
		}
			
		if(leftCurrent > maxCurrent) {
			maxCurrent = leftCurrent;
		}
		if(rightCurrent > maxCurrent) {
			maxCurrent = rightCurrent;
		}
	}
	Serial.print("Max current reading: ");
	Serial.println(maxCurrent);
	if(robot->isStalled()) {
		Serial.println("We stalled.");
	}
	
	robot->stop();
}

int testPanServoForFire() {
	int degrees = 90;
	boolean fireFound = false;
	
	robot->setFanServo(degrees);
	delay(750);	// let it get all the way there!
	delay(500); 	// and, let fire sensor settle down --VERY IMPORTANT!!
	
	while(degrees > -180) {
		delay(60);
		degrees -= 5;
		robot->setFanServo(degrees);
		
		if(robot->isFire(false)) {
			// triple check
			delay(100);
			if(robot->isFire(false)) {
				delay(100);
				if(robot->isFire(false)) {
					fireFound = true;
					break;
				}
			}
		}
	}	
	
	if(fireFound) {		
		Serial.print("Initial fire found at ");
		Serial.println(degrees);
		Serial.print("Fire reading: ");
		Serial.println(robot->getFireReading());
	}	
	
	short fireDegrees = degrees;
	
	// scan to find minimum
	if(fireFound) {
		delay(500);
		float reading = robot->getFireReading();
		float minimumReading = reading;
		Serial.print("Initial reading: ");
		Serial.println(reading);
		do {
			degrees -= 2;
			robot->setFanServo(degrees);
			delay(15);
			reading = robot->getFireReading();
		
			if(reading < minimumReading) {
				// double check
				delay(30);
				reading = robot->getFireReading();
				if(reading < minimumReading) {
				
					minimumReading = reading;
					fireDegrees = degrees;
					
					Serial.print("New minimum reading: ");
					Serial.println(minimumReading);
				}
			}
			
		} while(degrees > -180);
	}
	
	if(fireFound) {
		Serial.print("Fire found at ");
		Serial.println(fireDegrees);
		robot->setFanServo(fireDegrees);
		delay(2000);
		return fireDegrees;
	}	
	
	robot->setFanServo(0);
	return ROBOT_NO_FIRE_FOUND;	// NOT 0 as we may be facing the fire!
}

void testPutOutFire() {
	int degrees = robot->panServoForFire();
	if(degrees != ROBOT_NO_FIRE_FOUND) {
		Serial.print("degrees to turn: ");
		Serial.println(degrees);
		
		// let me see what's happening
		delay(2000);
	
		// face the right direction
		robot->setFanServo(0);
		robot->turn(degrees);
		delay(500);
		
		robot->fightFire();
	}
}

void testApproachAndPan() {
	/* dangerous without stall protection
  robot->drive(50, 50);
  while((robot->getFrontWallDistance() < 15) {
    delay(75);
  } */
  robot->stop();  
  robot->turn(90);
  delay(500);
  robot->turn(-180);
  delay(500);
  robot->turn(180);
  delay(500);
  robot->turn(-180);
  delay(500);
  robot->turn(180);
  delay(500);
  robot->turn(-90);
}

};

#endif
