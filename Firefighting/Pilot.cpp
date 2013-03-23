#include <Pilot.h>

#define DEBUG_DELAY 0

Pilot::Pilot(Maze* inMaze, FirefighterRobot* inRobot, mazeHeading startHeading){
	maze = inMaze;
	robot = inRobot;
	heading = startHeading;
	pathIndex = 0;
	lastPingTime = 0;

	followMethod = PILOT_FOLLOW_NONE;
	nodeCheck = PILOT_CHECK_FORWARD;
}

void Pilot::setStart(short startPathIndex, mazeHeading startHeading) {
	pathIndex = startPathIndex;
	heading = startHeading;
}

boolean Pilot::fightFire() {
	Serial.println("Checking for fire!");
	int degrees = robot->panServoForFire();
	if(degrees != ROBOT_NO_FIRE_FOUND) {
		Serial.print("degrees to turn: ");
		Serial.println(degrees);
		
		// let me see what's happening
		// delay(2000);
	
		// face the right direction
		robot->setFanServo(0);
		robot->turn(degrees * DEG_TO_RAD);
		delay(500);
		
		robot->fightFire();
		return true;
	}
	else {
		return false;
	}
}

/**
 *	Figure out where we are going next.
 *
 **/
int Pilot::setCourse() {
	currentNode = maze->getPathNode(pathIndex);
	if(currentNode.isRoom) {
		boolean success = fightFire();
		if(success) {
			return PILOT_FIRE_EXTINGUISHED;
		}
	}

	if(pathIndex >= maze->getPathLength() - 1) {
		// we are at the last node..and failed to find the fire, presumably.
		return -1;
	}

	nextNode = maze->getPathNode(pathIndex + 1);
	
	Serial.print("At path index ");
	Serial.println(pathIndex);
	Serial.print("Current node id: ");
	Serial.println(currentNode.id);
	Serial.print("Next node id: ");
	Serial.println(nextNode.id);
	
	boolean doAlignLeft = false;
	boolean doAlignRight = false;
	
	// figure out which direction to get to the next node
	mazeHeading newHeading;
	float nodeDistance = -1;
	for(int i = 0; i < 4; i++) {
		if(currentNode.neighbor[ i ] == nextNode.id) {
			newHeading = (mazeHeading)i;
			nodeDistance = currentNode.distToNeighbor[ i ];
			break;
		}
	}
	
	if(nodeDistance < 0) {
		// invalid path
		Serial.print("Can't find neighbor path for current path index ");
		Serial.println(pathIndex);
		return -2;
	}
	
	// we can't set hallway width in the constructor, as maze is not ready yet
	hallwayWidth = maze->getHallwayWidth();
	if(hallwayWidth == 0) {
		Serial.println("Hallway width was zero!!! Corrupted memory, exiting.");
		return -2;
	}

	// TODO: rooms 3 and 4 may have a hall of width 56 cm between them
	if((currentNode.id == 0) && (nextNode.id == 1)) {
		hallwayWidth = 34;	// this applies when we are heading for room 3
	}
	
	// TODO: is this right? I don't think there is a wall there
	// hand-tune for entering room 2
	if((currentNode.id == 4) && (nextNode.id == 7)) {
		robot->resetOdometers();
		robot->resetStallWatcher();
		robot->markPosition();
		robot->updateOdometry();
		
		// ((46.0 + 8.5)/2.0) - 6.25
		float nudgeDistance = 21.25;
		Serial.print("Nudging along wall ");
		Serial.println(nudgeDistance);		
		while((robot->getDistanceFromMarkedPoint() < nudgeDistance) && (!robot->isStalled())) {
			robot->followWall(ROBOT_LEFT);
			robot->updateOdometry();
		}
		
		// unlikely...
		if(robot->isStalled()) {	
			robot->stop();	
			// TODO: recovery code is incorrect here
			robot->recover();
			delay(500);
			robot->align(ROBOT_LEFT);
			delay(500);
			
			robot->markPosition();
			robot->updateOdometry();

			while((robot->getDistanceFromMarkedPoint() < nudgeDistance) && (!robot->isStalled())) {
				robot->followWall(ROBOT_LEFT);
				robot->updateOdometry();
			}			
		}
		
		robot->stop();
	}
	// hand-tune for *after* leaving node 4 to go to 1
	if((currentNode.id == 1) && (nextNode.id == 8)) {
		// nudgeForward();
		float nudgeDistance = ((hallwayWidth + robot->getTrackWidth())/2.0);
		robot->move(nudgeDistance, 0);
		robot->stop();
	}
	
	// hand-tuning for going from node 2 to node 3
	if((currentNode.id == 2) && (nextNode.id == 3)) {
		robot->align(ROBOT_LEFT);
		robot->stop();
	}
	
	// turn the robot the right direction and update heading
	changeHeading(heading, newHeading);
	
	mazeHeading right = (mazeHeading)((newHeading + 1) % 4);
	mazeHeading left = (mazeHeading)((newHeading + 3) % 4);		
			
	// default wall follow method is left wall
	followMethod = PILOT_FOLLOW_LEFT;
	if( currentNode.neighbor[ left ] != MAZE_WALL) {
		if( currentNode.neighbor[ right ] == MAZE_WALL) {
			followMethod = PILOT_FOLLOW_RIGHT;
		}
		else {
			// we are in a node with no walls on either side...look at next one
			if( nextNode.neighbor[ left ] != MAZE_WALL) {
				if( nextNode.neighbor[ right ] == MAZE_WALL) {
					followMethod = PILOT_FOLLOW_RIGHT;
				}
			}
		}
	}
		
		// how do we check we've gotten there?
		// forward wall detection is safest
		if( nextNode.neighbor[ newHeading ] == MAZE_WALL ) {
			// set flag to check for forward wall appearance
			nodeCheck = PILOT_CHECK_FORWARD;
		}
		else if(( currentNode.neighbor[ right ] == MAZE_WALL) && ( nextNode.neighbor[ right ] != MAZE_WALL)) {
			// set flag for right wall loss
			nodeCheck = PILOT_CHECK_RIGHT;
		}
		else if(( currentNode.neighbor[ left ] == MAZE_WALL) && ( nextNode.neighbor[ left ] != MAZE_WALL)) {
			// set flags for left wall loss
			nodeCheck = PILOT_CHECK_LEFT;
		}
//		else if(( currentNode.neighbor[ right ] != MAZE_WALL) && ( nextNode.neighbor[ right ] == MAZE_WALL)) {
//			// set flags for right wall appearance (may be used for that one problem room)
//			// TODO: uncomment when this is implemented
//			nodeCheck = PILOT_CHECK_RIGHT_APPEARANCE;
//		}
		else if((currentNode.id == 6) && (nextNode.id == 4)) {
			nodeCheck = PILOT_CHECK_RIGHT;
			followMethod = PILOT_FOLLOW_LEFT;
			doAlignLeft = true;
		}
		else {
			// this shouldn't happen; unless we add wall appearance as a node option
			Serial.print("Can't find right check to use for next node, current index is ");
			Serial.println(pathIndex);
			return -2;
		}
				
	// update current node to new one
	currentNode.isVisited = true;
	distanceToNext = nodeDistance;
	
	// for rooms, follow left wall in
	if((nextNode.isRoom) && (nextNode.id != 7)) {
		Serial.println("Next node is a room.");
		nodeCheck = PILOT_CHECK_DISTANCE;
		followMethod = PILOT_FOLLOW_LEFT;
		distanceToNext = hallwayWidth + 5;
	}
	
	boolean useSonarAlignment = true;
	if(useSonarAlignment) {
		Serial.println("Using sonar alignment.");
		if(followMethod == PILOT_FOLLOW_RIGHT) {
			doAlignRight = true;
		}
		else if(followMethod == PILOT_FOLLOW_LEFT) {
			doAlignLeft = true;
		}
	}
	else {
		// manually put in ones we'll need
		if((currentNode.id == 5) && (nextNode.id == 6)) {
			Serial.println("Manual serial alignment.");
			doAlignLeft = true;
		}
	}

	// (put any handtuning here)
	if(pathIndex == 0) {
		doAlignLeft = false;
		doAlignRight = false;
	}
	if((currentNode.id == 1) && (nextNode.id == 2)) {
		followMethod = PILOT_FOLLOW_LEFT;
		doAlignLeft = true;
	}	
	if((currentNode.id == 1) && (nextNode.id == 8)) {
		followMethod = PILOT_FOLLOW_RIGHT;
		doAlignRight = true;
	}
	if(currentNode.isRoom) {
		// we are following right wall out
		followMethod = PILOT_FOLLOW_RIGHT;
		doAlignRight = true;
	}
	if(nextNode.id == 7) {
		nodeCheck = PILOT_CHECK_FORWARD;
		followMethod = PILOT_FOLLOW_NONE;
		doAlignRight = false;
		doAlignLeft = false;
	}

	// if we will be following a wall, make sure that it is in view first.
	if(doAlignRight || doAlignLeft) {
		short wallSide = ROBOT_RIGHT;
		if(!doAlignRight) {
			wallSide = ROBOT_LEFT;
		}

		Serial.print("Aligning side ");
		Serial.println(wallSide);

		int heading;
		if(wallSide == ROBOT_LEFT ) {
			heading = (newHeading + 3) % 4; // left heading
		}
		else {
			heading = (newHeading + 1) % 4; // right heading
		}

		if(currentNode.neighbor[heading] != MAZE_WALL) {
			Serial.println("Moving forward to find wall.");
						
			float dist = nudgeToAlign(wallSide);
			distanceToNext -= dist;
			if(dist != 0) {
				delay(500); // need this for wall alignment.
			}
		}
		
		// And now. Try to align with wall.
		robot->align(wallSide);
		delay(500);	// delays after turns are important
	}

	if(followMethod == PILOT_FOLLOW_NONE) {
		// set a goal so we drive forward
		robot->setGoal(distanceToNext + maze->getNodeRadius() + 50, 0);
	}

	/*
	 * Originally this was done for EVERY node. I'd rather do this once at the start,
	 * while we are lined up nicely; however, the disadvantage is that I will only be
	 * able to set one IR sensor at that time, and assume the other is the same (which
	 * testing bears out to be valid).
	 */
	if(pathIndex == 0) {
		boolean isDesiredWallSensorReadingSet = false;

//		if(followMethod == PILOT_FOLLOW_LEFT) {
//			Serial.println("Initializing wall sensor on left.");
//			robot->initDesiredIRSensorReadings(ROBOT_LEFT);
//			isDesiredWallSensorReadingSet = true;
//		}
//		else if(followMethod == PILOT_FOLLOW_RIGHT) {
//			Serial.println("Initializing wall sensor on right.");
//			robot->initDesiredIRSensorReadings(ROBOT_RIGHT);
//			isDesiredWallSensorReadingSet = true;
//		}

		if(!isDesiredWallSensorReadingSet) {
			Serial.println("WARNING: Wall sensors have not been initialized! Using defaults.");
		}
	}
		
	robot->resetOdometers();
	robot->resetStallWatcher();
	// done so getDistanceTravelled is accurate:
	robot->markPosition();
	robot->updateOdometry();
	// make sure we start at a reasonable speed
	robot->resetCalculatedMovePWMs();
	
	Serial.print("For path index ");
	Serial.println(pathIndex);
	Serial.print("Follow method is ");
	Serial.println(followMethod);
	Serial.print("Check method is ");
	Serial.println(nodeCheck);
	Serial.print("Distance to next node is ");
	Serial.println(distanceToNext);
	// Serial.print("Front stop distance is ");
	// Serial.println(frontStopDistance);
	
	pathIndex++;

	return 0;
}

void Pilot::changeHeading(mazeHeading currentHeading, mazeHeading newHeading) {
	int delta = (newHeading - currentHeading + 4) % 4;
	
	if(delta != 0) {
		Serial.print("Changing heading from ");
		Serial.print(currentHeading);
		Serial.print(" to ");
		Serial.println(newHeading);
		
		robot->stop();
		delay(500);

		float my90 = 90 * DEG_TO_RAD;
	
		if(delta == 1) {
			// new heading is to the right
			robot->turn(-my90);
		}
		else if(delta == 3) {
			// new heading is to the left
			robot->turn(my90);
		}
		else {
			// turn around
			robot->turn(90+my90);
		}
		// let motors settle
		robot->stop();
		delay(3 * 500); // important!
	}
	else {
		Serial.println("No heading change.");
	}

	heading = newHeading;
}

int Pilot::go() {	
	// if we are following a wall or moving and haven't reached full power (use fabs), keep going
	if(robot->isStalled()) {
		boolean doRecovery = true;
		if((followMethod == PILOT_FOLLOW_LEFT) || (followMethod == PILOT_FOLLOW_RIGHT)) {
			if(fabs(robot->getFollowWallCalculatedPWM()) < robot->getMaxAllowedPWM()) {
				doRecovery = false;
			}
			else {
				if(fabs(robot->getMoveCalculatedPWM()) < robot->getMaxAllowedPWM()) {
					doRecovery = false;
				}
			}
		}
		robot->resetStallWatcher();

		if(doRecovery) {
			robot->recover();

			if(followMethod == PILOT_FOLLOW_NONE) {
				// re-orient goal point
				robot->setGoal(distanceToNext + maze->getNodeRadius() + 50, 0);
			}
		}
	}

	// We must make the distance trigger far enough that we do not simply follow the
	// wall around the turn!
	float bufferDistance = 8;
	const float desiredWallDistance = ((hallwayWidth - robot->getTrackWidth()) / 2.0) + 2;
	
	robot->updateOdometry();
	float distanceTravelled = robot->getDistanceFromMarkedPoint();
	
	// straight distance check first
	if(nodeCheck == PILOT_CHECK_DISTANCE) {
		if((distanceTravelled + bufferDistance) > distanceToNext) {
				Serial.println("PILOT_CHECK_DISTANCE triggered.");
				robot->stop();
				delay(DEBUG_DELAY);
				return 1;			
		}
	}
	
	long sensorPause = 5;
	if(nodeCheck == PILOT_CHECK_FORWARD) {
		sensorPause = 100; // time for the ultrasonic reading itself, +50
	}
			
	// if we are approaching the next node
	// if(distanceTravelled + maze->getNodeRadius() > distanceToNext) {
			
		// if enough time since last sensor check has elapsed, do them
		long now = millis();
		if(now - lastPingTime > sensorPause) {
			lastPingTime = now;
	
			if(nodeCheck == PILOT_CHECK_FORWARD) {
				// if wall found, move up to 1/2 hallway width from it then stop, return 1					
				float wallDistance = robot->getFrontWallDistance();
										
				if(wallDistance < desiredWallDistance + bufferDistance) {
					delay(75);
					robot->updateOdometry();

					wallDistance = robot->getFrontWallDistance();
					lastPingTime += 75;
					if(wallDistance  < desiredWallDistance + bufferDistance) {
						robot->stop();													
						Serial.println("PILOT_CHECK_FORWARD triggered.");
						delay(500);	// don't remove this!

						// now we need to confirm that we didn't trigger based on a bad alignment
						// also, lines us up for the subsequent move
						boolean bAlignDone = false;
						if((followMethod == PILOT_FOLLOW_LEFT) || (followMethod == PILOT_FOLLOW_RIGHT)) {
							if(followMethod == PILOT_FOLLOW_LEFT) {
								bAlignDone = robot->align(ROBOT_LEFT);
							}
							else {
								bAlignDone = robot->align(ROBOT_RIGHT);
							}
							robot->stop();
						}

						wallDistance = robot->getFrontWallDistance();
						// if re-aligned, do one last check...
						if(bAlignDone) {
							if(wallDistance > desiredWallDistance + bufferDistance) {
								Serial.println("Alignment gives cancellation of front wall check.");
								return 0;
							}
						}
							
						Serial.print("Front wall distance ");
						Serial.println(wallDistance);
//						Serial.print("Desired walldistance ");
//						Serial.println(desiredWallDistance);

						// don't move for small things
						if(fabs(wallDistance - desiredWallDistance) < 5) {
							wallDistance = desiredWallDistance;
						}
						
						Serial.print("Moving ");
						Serial.println(wallDistance - desiredWallDistance);
						if(wallDistance > desiredWallDistance) {
							robot->move(wallDistance - desiredWallDistance, 0);
						}
						else if(wallDistance < desiredWallDistance) {
							robot->backUp(desiredWallDistance - wallDistance);
						}
						robot->stop();
						delay(DEBUG_DELAY);
						return 1;
					}
				}
			} // if checking forward
			
			float maxMisalignment = 5 * DEG_TO_RAD; // don't make this too small
			if((nodeCheck == PILOT_CHECK_LEFT) || (nodeCheck == PILOT_CHECK_RIGHT)) {
				short wallSide = ROBOT_LEFT;
				if(nodeCheck == PILOT_CHECK_RIGHT) {
					wallSide = ROBOT_RIGHT;
				}
				// if wall lost, delay slightly and check again to confirm, stop and return 1
				if(robot->isSideWallLost(wallSide)) {
					delay(50);
					robot->updateOdometry();

					lastPingTime += 50;
					if(robot->isSideWallLost(wallSide)) {
						Serial.println("PILOT_CHECK_LEFT/RIGHT triggered.");

						// see if we are way off in angle, and if so, try to align
						robot->stop();
						delay(500);
						robot->resetStallWatcher();

						boolean bTurnDone = false;
						float theta = robot->getMisalignmentAngle(wallSide);
						if((theta != ROBOT_NO_VALID_DATA) && (fabs(theta) > maxMisalignment)) {
							// this should work for both right and left sides
							Serial.print("Turning to find wall (degrees): ");
							Serial.println(theta * RAD_TO_DEG);
							robot->turn(theta);
							robot->stop();

							bTurnDone = true;
						}

						// if no turn, or turned but wall still lost, give up
						if((!bTurnDone) || (robot->isSideWallLost(wallSide))) {
							Serial.print("Lost wall: ");
							Serial.println(wallSide);
							nudgeForwardAfterWallLoss(wallSide);
							return 1;
						}
						else {
							Serial.println("Re-found wall.");

							// try to avoid endless loop
							lastPingTime += 300;
						}
					}
				}
			} // if checking wall still present

//			if(nodeCheck == PILOT_CHECK_RIGHT_APPEARANCE) {
//				// TODO: implement. if right wall appears, delay slightly and check again to confirm, stop and return 1
//				// until implemented, this is an error
//				Serial.println("PILOT_CHECK_RIGHT_APPEARANCE is not implemented yet!!!!!");
//				return -2;
//			}
			
		} // if enough time elapsed
	// } // endif approaching next node
	
	// we are still here, so let's move on
	if(followMethod == PILOT_FOLLOW_LEFT) {
		robot->followWall(ROBOT_LEFT);
	}
	else if(followMethod == PILOT_FOLLOW_RIGHT) {
		robot->followWall(ROBOT_RIGHT);
	}
	else if(followMethod == PILOT_FOLLOW_NONE) {
		robot->driveTowardGoal();
	}
	else {
		// error
		return -2;
	}
	
	return 0;
}

/**
 *	Get robot into middle of next hallway. We need to know exactly
 *	how far away (along x) the robot is from the end of the wall. This
 *	will depend on the distance from the wall and the angle of the robot
 *	with respect to the wall, as well as parameters relating to the placement
 *	of various sensors.
 */
void Pilot::nudgeForwardAfterWallLoss(short wallDirection) {
	robot->stop();
	delay(500);
	robot->resetCalculatedMovePWMs();

	float nudgeDistance = robot->getCalculatedWallEnd(wallDirection) + (hallwayWidth / 2.0);

	// TODO: check we haven't banged into anything... moveWithRecovery?
	robot->move(nudgeDistance, 0);
	delay(DEBUG_DELAY);
	
	Serial.print("Nudged forward ");
	Serial.println(nudgeDistance);
}

/**
 *	Move robot to where both sonar can see the wall it needs to align with. It's very important
 *	to make sure that we are on course here.
 */
float Pilot::nudgeToAlign(short wallDirection) {
	Serial.println("nudgeToAlign called.");

	float distance = 0;

	robot->resetOdometers();
	robot->resetStallWatcher();

	robot->markPosition();
	robot->setGoal(50, 0);

	long lastPingTime = 0;
	long sensorPause = 75;
	long now = lastPingTime;
	float minAllowedFrontWallDistance = 15.0;
	double minTravelRequired = (hallwayWidth / 2.0) + 5.0;

	boolean doLoop = true;
	float angleTurned = 0;

	while(doLoop) {
		if(robot->isStalled()) {
			if(fabs(robot->getMoveCalculatedPWM()) < robot->getMaxAllowedPWM()) {
				robot->resetStallWatcher();
			}
			else {
				robot->recover();
				robot->stop();

				// so goal point is re-oriented properly
				robot->setGoal(50, 0);
			}
		}

		// drive slowly forward
		robot->driveTowardGoal(0.5);

		// can't use getSideWallDistance, it tries repeatedly so is too slow
		// (even this version takes 150ms)
		if(robot->getDistanceFromMarkedPoint() > minTravelRequired) {
			if(robot->isAlignmentPossible(wallDirection, hallwayWidth * 1.4)) {
				Serial.println("Alignment possible.");
				doLoop = false;
				break;
			}
		}

		now = millis();
		if(now - lastPingTime > sensorPause) {
			lastPingTime = now;

			// are we about to hit a wall?
			float frontWallDistance = robot->getFrontWallDistance();
			if(frontWallDistance < minAllowedFrontWallDistance) {

				// Yes, we are. So, figure out which side is closer to wall, and turn away
				robot->stop();
				Serial.println("nudgeToAlign has encountered a forward obstacle");
				delay(500);

				angleTurned += robot->recover();
				Serial.print(angleTurned);

				// so goal point is re-oriented properly
				robot->setGoal(50, 0);

				// we may have been on an outside corner..in which case, should
				// move forward a while before testing alignment again
				minTravelRequired = robot->getDistanceFromMarkedPoint() +
						(minAllowedFrontWallDistance * 1.4 * 1.3);

				lastPingTime += 100;

			} // if forward wall detected
		} // if time to check sensors
	} // while looking for wall
	
	robot->updateOdometry();
	distance += robot->getDistanceFromMarkedPoint();

	// go further, because sonar range is way too wide and we may not be fully at the wall yet
	float distanceToContinue = robot->getTrackWidth();
	if(!robot->isStalled()) {
		robot->updateOdometry();
		robot->markPosition();

		while(!robot->isStalled()) {
			robot->followWall(wallDirection);
			if (robot->getDistanceFromMarkedPoint() > distanceToContinue) {
				break;
			}
			delay(10);
		}

		robot->updateOdometry();
		distance += robot->getDistanceFromMarkedPoint();
	}
	
	robot->stop();
	
	// TODO: if watcher stalled, do some recovery

	return distance;
}
