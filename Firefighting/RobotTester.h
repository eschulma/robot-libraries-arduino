#ifndef RobotTester_h
#define RobotTester_h

#include "Arduino.h"
#include <Motor.h>
#include <ControllerMotor.h>
#include <FireCheetah.h>
#include <Pilot.h>

#include <StandardCplusplus.h>
#include <serstream>

using namespace std;

extern ohserialstream serial;

class RobotTester {
	private:
		FireCheetah* robot;

	public:
		RobotTester(FireCheetah *inRobot) { robot = inRobot; };
	
void testWallLoss() {
  for(int i = 0; i < 400; i++) {
    boolean lost = robot->isSideWallLost(ROBOT_RIGHT);
    Serial.println(lost);
    delay(250);
  }
}

void testWallFollowingPilot() {
	// can use pilot.setStart to start at arbitrary node
	pilot.setCourse();

	// pilot.distanceToNext = 0; // smooth surfaces only!
	// Node radius is 30
	pilot.distanceToNext = 60;

	while(1) {
	    int returnCode = pilot.go();
	    if(returnCode != 0) {
	    	serial << F("Pilot return code ") << returnCode << endl;
	    	break;
	    }
	}
	robot->stop();
}

void testNudgeToAlign() {
	// can use pilot.setStart to start at arbitrary node
	pilot.setCourse();

	// pilot.distanceToNext = 0; // smooth surfaces only!
	// Node radius is 30
	pilot.distanceToNext = 60;
	serial << F("Hallway width: ") << pilot.hallwayWidth << endl;

	float distance = pilot.nudgeToAlign(ROBOT_LEFT);
	Serial.println(distance);

	robot->stop();
	delay(500);
	robot->align(ROBOT_LEFT);
}

void testSegment() {
  // can use pilot.setStart to start at arbitrary node; pathIndex, heading
  // do not start in a room with a candle, as extra call here to setCourse will cause problems
  pilot.setStart(13, MAZE_WEST);
  pilot.setCourse();

  boolean bContinue = true;
  while(bContinue) {
	  int returnCode = pilot.go();
	      if(returnCode > 0) {
	        returnCode = pilot.setCourse();
	        if(returnCode == PILOT_FIRE_EXTINGUISHED) {
	          robot->stop();
	          serial << F("Success!!!") << endl;

	          // put out the candle, now go back to start
	          returnCode = pilot.headHome();
	          if(returnCode < 0) {
	  			robot->stop();
	  			serial << F("Unable to head home, return code was ") << returnCode << endl;
	          	bContinue = false;
	          }
	          else {
	  			// set course for heading home before go is called again
	  			returnCode = pilot.setCourse();
	  			if(returnCode < 0) {
	  				robot->stop();
	  				serial << F("pilot.setCourse returned code ") << returnCode << endl;
	  				bContinue = false;
	  			}
	          }
	        }
	        else if(returnCode == PILOT_RETURNED_HOME) {
	          robot->stop();
	          serial << F("Back home!!!") << endl;
	          bContinue = false;
	        }
	        else if(returnCode < 0) {
	          robot->stop();
	          serial << F("pilot.setCourse returned code ") << returnCode << endl;
	          bContinue = false;
	        }
	      }
	      else if(returnCode < 0) {
	        robot->stop();
	        serial << F("pilot.go returned code ") << returnCode << endl;
	        bContinue = false;
	      }
  }
  robot->stop();
}


void testWallFollowing() {
  pilot.setCourse();

  int wallSide = ROBOT_RIGHT;
  float stopFrontDistance = 23 - ((robot->getTrackWidth())/2.0);

  robot->initDesiredIRSensorReadings(wallSide);
  robot->resetOdometers();
  robot->resetStallWatcher();
  
  while(1) {
	if(robot->isSideWallLost(wallSide)) {
		delay(50);
		if(robot->isSideWallLost(wallSide)) {
			break;
		}
	}
	// don't give up with stalls until we've reached maximum torque
	if(robot->isStalled()) {
		if(robot->getFollowWallCalculatedPWM() > robot->getMaxAllowedPWM()) {
			serial << F("We stalled!") << endl;
			break;
		}
	}
	float frontWallDistance = robot->getFrontWallDistance();
	if((frontWallDistance > 0) && (frontWallDistance < stopFrontDistance)) {
		serial << F("Forward wall detected.") << endl;
		break;
	}
    robot->followWall(wallSide);
    delay(50);	// so pings don't collide
  }    
  float lossDistanceReading = robot->getSideWallDistanceReading(wallSide);
  robot->stop();

  serial << F("Lost wall at IR side wall distance ") << lossDistanceReading << endl;
  serial << F("Front wall distance ") << robot->getFrontWallDistance() << endl;

  float val = robot->getCalculatedWallEnd(wallSide);
  serial << F("IR point along wall: ") << val << endl;
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

		serial << F("left ticks: ") << leftTicks << F("     right ticks: ") << rightTicks << endl;
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

	serial << F("left ticks: ") << ticks[0] << F("     right ticks: ") << ticks[1] << endl;

	delay(500);
	ticks[0] = robot->getOdometerValue(ROBOT_LEFT);
	ticks[1] = robot->getOdometerValue(ROBOT_RIGHT);

	serial << endl << F("1/2s after stop --") << endl;
	serial << F("left ticks: ") << ticks[0] << F("     right ticks: ") << ticks[1] << endl;

	delay(2000);
	ticks[0] = robot->getOdometerValue(ROBOT_LEFT);
	ticks[1] = robot->getOdometerValue(ROBOT_RIGHT);

	serial << endl << F("More than 2s after stop --") << endl;
	serial << F("left ticks: ") << ticks[0] << F("     right ticks: ") << ticks[1] << endl;
}

void testCircling() {
	robot->drive(-100, 100);
}

void testTurning() {
  robot->resetOdometers();

  robot->turn(-PI/2.0);
  robot->stop();
  serial << F("Ticks right after stop: ") << endl;
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
  serial << F("Current position: ") << robot->odom.getX() << F("     ") << robot->odom.getY() << endl;

  serial << F("Current pose (degrees): ") << robot->odom.getHeading() * RAD_TO_DEG << endl;
}

void printEncoderTicks() {
	long leftTicks = robot->getOdometerValue(ROBOT_LEFT);
	long rightTicks = robot->getOdometerValue(ROBOT_RIGHT);

	serial << F("left ticks: ") << leftTicks << F("     right ticks: ") << rightTicks << endl;
}

void testGoToGoal() {
  robot->resetOdometers();

  printPosition();
  robot->goToGoal(20, -20);
  printPosition();
  
  robot->stop();
  
  delay(2000);
  serial << endl << F("After first goal: ") << endl;
  printPosition();
  printEncoderTicks();

  robot->goToGoal(40, 0);
  robot->stop();
  printPosition();
}

void testFrontSonar() {
	while(1) {
		delay(250);
		serial << robot->getFrontWallDistance() << endl;
	}
}

void testSonar() {
	while(1) {
		float distanceLF = robot->sonar[SONAR_LEFT_F]->ping_cm();
		float distanceRF = robot->sonar[SONAR_RIGHT_R]->ping_cm();
		delay(100);
		float distanceLR = robot->sonar[SONAR_LEFT_R]->ping_cm();
		float distanceRR = robot->sonar[SONAR_RIGHT_R]->ping_cm();
		float distanceF = robot->sonar[SONAR_FRONT]->ping_cm();
		delay(100);

		serial << distanceF << F("  |  ") << distanceLF << F("    ")
				<< distanceLR << F("  |  ") << distanceRF << F("    ") << distanceRR << endl;
	}
}

void testIR() {
  while(1) {
    delay(250);
    serial << analogRead(WALL_LEFT_SENSOR_PIN) << F(" ") << analogRead(WALL_RIGHT_SENSOR_PIN) << endl;
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
  	  serial << F("distance diff: ") << diff << endl;
    if(diff != 0) {
	    float theta = atan2(diff, 7.7) * (180.0 / M_PI);
  	  serial << F("angle: ") << theta << endl;
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
			serial << leftCurrent << F(" ") << rightCurrent << endl;
		}
			
		if(leftCurrent > maxCurrent) {
			maxCurrent = leftCurrent;
		}
		if(rightCurrent > maxCurrent) {
			maxCurrent = rightCurrent;
		}
	}
	serial << F("Max current reading: ") << maxCurrent << endl;
	if(robot->isStalled()) {
		serial << F("We stalled.") << endl;
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
		
		if(robot->isFire()) {
			// triple check
			delay(100);
			if(robot->isFire()) {
				delay(100);
				if(robot->isFire()) {
					fireFound = true;
					break;
				}
			}
		}
		delay(100);
		serial << robot->getFireReading() << endl;
	}	
	
	if(fireFound) {		
		serial << F("Initial fire found at ") << degrees << F("Fire reading: ") << robot->getFireReading() << endl;
	}	
	else {
		serial << F("No fire found!") << endl;
	}
	
	short fireDegrees = degrees;
	
	// scan to find minimum
	// if(fireFound) {
		delay(500);
		float reading = robot->getFireReading();
		float minimumReading = reading;
		serial << F("Initial reading: ") << reading << endl;
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
					
					serial << F("New minimum reading: ") << minimumReading << endl;
				}
			}
			
		} while(degrees > -180);
	// }
	
	if(fireFound) {
		serial << F("Fire found at ") << fireDegrees << endl;
		robot->setFanServo(fireDegrees);
		delay(2000);
		return fireDegrees;
	}	
	else {
		serial << F("No fire found.") << endl;
	}
	
	robot->setFanServo(0);
	return ROBOT_NO_FIRE_FOUND;	// NOT 0 as we may be facing the fire!
}

void testPutOutFire() {
	int degrees = robot->panServoForFire();
	if(degrees != ROBOT_NO_FIRE_FOUND) {
		robot->fightFire(degrees);

//		while(robot->isFire()) {
//			robot->fightFire();
//		}
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
  robot->stop();
  delay(500);
  robot->turn(-180 * DEG_TO_RAD);
  robot->stop();
  delay(500);
  robot->turn(180 * DEG_TO_RAD);
  robot->stop();
  delay(500);
  robot->turn(-180 * DEG_TO_RAD);
  robot->stop();
  delay(500);
  robot->turn(180 * DEG_TO_RAD);
  robot->stop();
  delay(500);
  robot->turn(-90 * DEG_TO_RAD);
  robot->stop();
}

};

#endif
