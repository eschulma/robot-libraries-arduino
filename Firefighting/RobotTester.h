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
  int wallSide = ROBOT_LEFT;
  float stopFrontDistance = 23 - ((robot->getTrackWidth())/2.0);

  robot->initDesiredWallSensorReadings(wallSide);
  robot->resetOdometers();
  robot->resetStallWatcher();
  
  while(!robot->isSideWallLost(wallSide)) {
	// don't give up with stalls until we've reached maximum torque
	if(robot->isStalled()) {
		if(robot->getFollowWallCalculatedSpeed() > 200) {
			Serial.println("We stalled!");
			break;
		}
	}
	float frontWallDistance = robot->getFrontWallDistance();
	if((frontWallDistance > 0) && (frontWallDistance < stopFrontDistance)) {
		Serial.println("Forward wall detected.");
		break;
	}
    robot->followWall(wallSide);
    delay(50);	// so pings don't collide
  }    
  float lossDistanceReading = robot->getSideWallDistanceReading(wallSide);
  robot->stop();

  Serial.print("Lost wall at side wall distance ");
  Serial.println(lossDistanceReading);
  Serial.print("Front wall distance ");
  Serial.println(robot->getFrontWallDistance());
  Serial.print("IR point along wall: ");
  Serial.println(robot->getIRWallForwardDistance(wallSide));
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
	robot->drive(100, 100);

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
  long leftTicks, rightTicks;
  robot->resetOdometers();

  robot->turn(PI/2.0);
  robot->stop();
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  Serial.println("Ticks right after stop: ");
  printEncoderTicks();

  /* delay(2000);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  Serial.println("Ticks after delay: ");
  printEncoderTicks();

  robot->turn(-PI/2.0);
  robot->stop();

  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  Serial.println("Ticks right after stop: ");
  printEncoderTicks();

  delay(2000);
  rightTicks = robot->getOdometerValue(ROBOT_RIGHT);
  leftTicks = robot->getOdometerValue(ROBOT_LEFT);
  printEncoderTicks(); */

  robot->stop();
}

void testMovingOld() {
	robot->resetOdometers();
	robot->backUp(15);
	robot->stop();
}

void testMoving() {
	printPosition();
	robot->move(20, 0);

	robot->stop();
	printPosition();

	delay(500);

	robot->move(0, 20);
	printPosition();

	robot->stop();
	printPosition();

	delay(500);
	printPosition();
}

void printPosition() {
  Serial.print("Current position: ");
  Serial.print(robot->odom.getX());
  Serial.print("     ");
  Serial.println(robot->odom.getY());

  Serial.print("Current pose (degrees): ");
  Serial.println(robot->odom.getHeading() * RAD_TO_DEG);
}

void printEncoderTicks() {
	  long leftTicks = robot->getOdometerValue(ROBOT_LEFT);
	  long rightTicks = robot->getOdometerValue(ROBOT_RIGHT);

	  Serial.print("left ticks: ");
	  Serial.print(leftTicks);
	  Serial.print("     right ticks: ");
	  Serial.println(rightTicks);
}

void testGoToGoal() {
  robot->resetOdometers();

  printPosition();
  robot->goToGoal(20, -20);
  printPosition();
  
  robot->stop();
  
  delay(2000);
  Serial.println("\nAfter first goal: ");
  printPosition();
  printEncoderTicks();

  robot->goToGoal(40, 0);
  robot->stop();
  printPosition();
}

void testFrontSonar() {
	while(1) {
		delay(250);
		Serial.println(robot->getFrontWallDistance());
	}
}

void testSonarPair() {
	Serial.println("Testing sonar right.");

	sonarLocation front = SONAR_RIGHT_F;
	sonarLocation rear = SONAR_RIGHT_R;

	while(1) {
		float distanceF = robot->sonar[front]->ping_cm();
		delay(100);
		float distanceR = robot->sonar[rear]->ping_cm();
		delay(100);

		Serial.print(distanceF);
		Serial.print("    ");
		Serial.println(distanceR);
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
  robot->align(ROBOT_RIGHT);
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
		robot->turn(degrees * DEG_TO_RAD);
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
  robot->turn(90 * DEG_TO_RAD);
  delay(500);
  robot->turn(-180 * DEG_TO_RAD);
  delay(500);
  robot->turn(180 * DEG_TO_RAD);
  delay(500);
  robot->turn(-180 * DEG_TO_RAD);
  delay(500);
  robot->turn(180 * DEG_TO_RAD);
  delay(500);
  robot->turn(-90 * DEG_TO_RAD);
}

};

#endif
