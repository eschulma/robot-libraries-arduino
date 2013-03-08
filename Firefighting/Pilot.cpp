#include <Pilot.h>

#define DEBUG_DELAY 0

Pilot::Pilot(Maze* inMaze, FirefighterRobot* inRobot, mazeHeading startHeading){
	maze = inMaze;
	robot = inRobot;
	heading = startHeading;
	pathIndex = 0;
	lastPingTime = 0;
	hallwayWidth = maze->getHallwayWidth();

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
	
	if(hallwayWidth == 0) {
		Serial.println("Hallway width was zero!!! Corrupted memory, exiting.");
		return -2;
	}

	// TODO: rooms 3 and 4 may have a hall of width 56 cm between them?
	if((currentNode.id == 0) && (nextNode.id == 1)) {
		hallwayWidth = 34;	// this applies when we are heading for room 3
	}
	else {
		hallwayWidth = maze->getHallwayWidth();
	}
	
	// hand-tune for entering room 2
	if((currentNode.id == 4) && (nextNode.id == 7)) {
		robot->resetOdometers();
		robot->resetStallWatcher();
		robot->odom.setGoalPosition(0, 0);
		robot->odom.update();
		
		// ((46.0 + 8.5)/2.0) - 6.25
		float nudgeDistance = 21.25;
		Serial.print("Nudging along wall ");
		Serial.println(nudgeDistance);		
		while((robot->odom.getDistanceToGoal() < nudgeDistance) && (!robot->isStalled())) {
			robot->followWall(ROBOT_LEFT);
			robot->odom.update();
		}
		
		// unlikely...
		if(robot->isStalled()) {	
			robot->stop();	
			robot->recover();
			delay(500);
			robot->alignLeft();
			delay(500);
			
			robot->odom.setGoalPosition(0, 0);
			robot->odom.update();

			while((robot->odom.getDistanceToGoal() < nudgeDistance) && (!robot->isStalled())) {
				robot->followWall(ROBOT_LEFT);
				robot->odom.update();
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
		robot->alignLeft();
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
		else if(( currentNode.neighbor[ right ] != MAZE_WALL) && ( nextNode.neighbor[ right ] == MAZE_WALL)) {
			// set flags for right wall appearance (may be used for that one problem room)
			nodeCheck = PILOT_CHECK_RIGHT_APPEARANCE;
		}
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
	pathIndex++;
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
	if((currentNode.id == 1) && (nextNode.id == 2)) {
		followMethod = PILOT_FOLLOW_LEFT;
		doAlignLeft = false;
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
		// TODO: don't use PILOT_FOLLOW_NONE, use a straight move command instead, or something to keep us straight
		followMethod = PILOT_FOLLOW_NONE;
		doAlignRight = false;
		doAlignLeft = false;
	}

	// if we will be following a wall, make sure that it is in view first.
	if(doAlignRight) {
		Serial.println("Aligning right.");
		int rightHeading = (newHeading + 1) % 4;
		if(currentNode.neighbor[rightHeading] != MAZE_WALL) {
			Serial.println("Moving forward to find right wall.");
						
			float dist = nudgeToAlign(MOTOR_RIGHT);
			distanceToNext -= dist;
			if(dist != 0) {
				delay(500); // need this for wall alignment.
			}
		}
		
		// And now. Try to align with wall.
		robot->alignRight();
		delay(500);	// delays after turns are important
	}
	else if(doAlignLeft) {
		Serial.println("Aligning left.");
		int leftHeading = (newHeading + 3) % 4;
		if(currentNode.neighbor[leftHeading] != MAZE_WALL) {
			Serial.println("Moving forward to find left wall.");
						
			float dist = nudgeToAlign(MOTOR_LEFT);
			distanceToNext -= dist;
			if(dist != 0) {
				delay(500); // need this for wall alignment.
			}
		}
		
		// And now. Try to align with wall.
		robot->alignLeft();
		delay(500);	// delays after turns are important
	}

	/*
	 * Originally this was done for EVERY node. I'd rather do this once at the start,
	 * while we are lined up nicely; however, the disadvantage is that I will only be
	 * able to set one IR sensor at that time, and assume the other is the same (which
	 * testing bears out to be valid).
	 */
	if(pathIndex == 0) {
		boolean isDesiredWallSensorReadingSet = false;

		// probably don't need this, but anyway...
		robot->stop();
		delay(500);

		if(followMethod == PILOT_FOLLOW_LEFT) {
			robot->initDesiredWallSensorReadings(ROBOT_LEFT);
			isDesiredWallSensorReadingSet = true;
			Serial.println("Initializing wall sensor on left.");
		}
		else if(followMethod == PILOT_FOLLOW_RIGHT) {
			robot->initDesiredWallSensorReadings(ROBOT_RIGHT);
			isDesiredWallSensorReadingSet = true;
			Serial.println("Initializing wall sensor on right.");
		}

		if(!isDesiredWallSensorReadingSet) {
			Serial.println("WARNING: Wall sensors have not been initialized! Using defaults.");
		}
	}
		
	robot->resetOdometers();
	robot->resetStallWatcher();
	// done so getDistanceTravelled is accurate:
	robot->odom.setGoalPosition(0,0);
	robot->odom.update();
	
	Serial.print("For path index ");
	Serial.println(pathIndex - 1);
	Serial.print("Follow method is ");
	Serial.println(followMethod);
	Serial.print("Check method is ");
	Serial.println(nodeCheck);
	Serial.print("Distance to next node is ");
	Serial.println(distanceToNext);
	// Serial.print("Front stop distance is ");
	// Serial.println(frontStopDistance);
	
	return 0;
}

void Pilot::changeHeading(mazeHeading currentHeading, mazeHeading newHeading) {
	int delta = (newHeading - currentHeading + 4) % 4;
	
	if(delta != 0) {
		Serial.print("Changing heading from ");
		Serial.print(currentHeading);
		Serial.print(" to ");
		Serial.print(newHeading);
		
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
	// TODO: if we are following a wall or moving and haven't reached full power yet, keep going
	if(robot->isStalled()) {
		robot->recover();
		robot->turn(-30 * DEG_TO_RAD);
		robot->stop();
	}

	// TODO: find replacement -- and moveStopDistance can be much smaller
	float moveStopDistance = 8;
	const float desiredWallDistance = ((46-robot->getTrackWidth()) / 2.0) + 2;
	
	if((followMethod == PILOT_FOLLOW_LEFT) || (followMethod == PILOT_FOLLOW_RIGHT)) {
		moveStopDistance += 10; // 2.0 * robot->getMoveStopDistance();
	}

	robot->odom.update();
	float distanceTravelled = fabs(robot->odom.getDistanceToGoal());
	
	// straight distance check first
	if(nodeCheck == PILOT_CHECK_DISTANCE) {
		if((distanceTravelled + moveStopDistance) > distanceToNext) {
				Serial.println("PILOT_CHECK_DISTANCE triggered.");
				robot->stop();
				delay(DEBUG_DELAY);
				return 1;			
		}
	}
	
	long sensorPause = 5;
	if(nodeCheck == PILOT_CHECK_FORWARD) {
		sensorPause = 100;
	}
			
	// if we are approaching the next node
	if(distanceTravelled + maze->getNodeRadius() > distanceToNext) {
			
		// if enough time since last sensor check has elapsed, do them
		long now = millis();
		if(now - lastPingTime > sensorPause) {
			lastPingTime = now;
	
			if(nodeCheck == PILOT_CHECK_FORWARD) {
				// if wall found, move up to 1/2 hallway width from it then stop, return 1					
				float wallDistance = robot->getFrontWallDistance();
										
				if(wallDistance < desiredWallDistance + (2.0 * moveStopDistance)) {
					delay(75);
					wallDistance = robot->getFrontWallDistance();
					lastPingTime += 75;
					if(wallDistance  < desiredWallDistance + (2.0 * moveStopDistance)) {
						robot->stop();													
						Serial.println("PILOT_CHECK_FORWARD triggered.");
						delay(500);	// don't remove this!		robot->stop(); // we are already going the right way!

						wallDistance = robot->getFrontWallDistance();
							
						Serial.print("Front wall distance ");
						Serial.println(wallDistance);
						Serial.print("Desired walldistance ");
						Serial.println(desiredWallDistance);
						
						if(desiredWallDistance < 0) {
							Serial.print("Hallway width ");
							Serial.println(hallwayWidth);
							Serial.print("Track width ");
							Serial.println(robot->getTrackWidth());
						}
						
						// don't move for small things
						if(fabs(wallDistance - desiredWallDistance) < 5) {
							wallDistance = desiredWallDistance;
						}
						
						// put in extra delay if we are reversing direction
						if(wallDistance < desiredWallDistance) {
							delay(500);
							// put in even more delay for higher speed
							if((followMethod == PILOT_FOLLOW_LEFT) || (followMethod == PILOT_FOLLOW_RIGHT)) {
								delay(500);
							}
						}
						
						Serial.print("Moving ");
						Serial.println(wallDistance - desiredWallDistance);
						robot->move(wallDistance - desiredWallDistance);
						delay(DEBUG_DELAY);
						return 1;
					}
				}
			} // if checking forward
			
			if(nodeCheck == PILOT_CHECK_LEFT) {
				// if left wall lost, delay slightly and check again to confirm, stop and return 1
				if(robot->isSideWallLost(ROBOT_LEFT)) {
					delay(50);
					lastPingTime += 50;
					if(robot->isSideWallLost(ROBOT_LEFT)) {
						Serial.println("PILOT_CHECK_LEFT triggered.");
						nudgeForwardAfterWallLoss(ROBOT_LEFT);
						return 1;
					}
				}
			}
			if(nodeCheck == PILOT_CHECK_RIGHT) {
				// if right wall lost, delay slightly and check again to confirm, stop and return 1
				if(robot->isSideWallLost(ROBOT_RIGHT)) {
					delay(50);
					lastPingTime += 50;
					if(robot->isSideWallLost(ROBOT_RIGHT)) {
						Serial.println("PILOT_CHECK_RIGHT triggered.");
						nudgeForwardAfterWallLoss(ROBOT_RIGHT);
						return 1;
					}
				}
			}
			if(nodeCheck == PILOT_CHECK_RIGHT_APPEARANCE) {
				// TODO: if right wall appears, delay slightly and check again to confirm, stop and return 1
			}
			
			/* if we are following a wall, make sure it is still there
			if(followMethod == PILOT_FOLLOW_LEFT) {
				if(robot->isSideWallLost(ROBOT_LEFT)) {
					delay(50);
					lastPingTime += 50;
					if(robot->isSideWallLost(ROBOT_LEFT)) {
						Serial.print("Lost our left wall to follow.");
						Serial.print(" Follow method value was ");
						Serial.println(followMethod);
						followMethod = PILOT_FOLLOW_NONE;
					}
				}
			}
			if(followMethod == PILOT_FOLLOW_RIGHT) {
				if(robot->isSideWallLost(ROBOT_RIGHT)) {
					delay(50);
					lastPingTime += 50;
					if(robot->isSideWallLost(ROBOT_RIGHT)) {
						Serial.println("Lost our right wall to follow.");
						followMethod = PILOT_FOLLOW_NONE;
					}
				}
			} */
			// TODO: add for rear wall follow, though this may be less necessary;reverse speed there too
			
		} // if enough time elapsed
	} // endif approaching next node
	
	// we are still here, so let's move on
	if(followMethod == PILOT_FOLLOW_LEFT) {
		robot->followWall(ROBOT_LEFT);
	}
	else if(followMethod == PILOT_FOLLOW_RIGHT) {
		robot->followWall(ROBOT_RIGHT);
	}
	else if(followMethod == PILOT_FOLLOW_LEFT_REAR) {
		robot->followWallRear(ROBOT_LEFT);
	}
	else if(followMethod == PILOT_FOLLOW_NONE) {
		// we lost our wall. Maybe go slower?
		robot->drive(robot->getMoveSpeed(), robot->getMoveSpeed());
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

	float nudgeDistance = robot->getIRWallForwardDistance(wallDirection) + (hallwayWidth / 2.0);

	if(wallDirection == ROBOT_LEFT) {
		robot->alignLeft();
	}
	else {
		robot->alignRight();
	}
	robot->move(nudgeDistance, 0);
	delay(DEBUG_DELAY);
	
	Serial.print("Nudged forward ");
	Serial.println(nudgeDistance);
}

/**
 *	Move robot to where both sonar can see the wall it needs to align with.
 */
float Pilot::nudgeToAlign(short wallDirection) {
	float distance = 0;
	
	robot->resetOdometers();
	robot->resetStallWatcher();
	while(!robot->isStalled()) {
		robot->drive(robot->getMoveSpeed(), robot->getMoveSpeed());		
		if(robot->getSideWallDistance(ROBOT_LEFT) < hallwayWidth) {
			// delay and double-check
			delay(75);
			if(robot->getSideWallDistance(ROBOT_LEFT) < hallwayWidth) {
				break;
			}
		}
	}
	
	// go a tad further, because sonar range is way too wide
	if(!robot->isStalled()) {
		robot->move(robot->getTrackWidth());
	}
	
	robot->stop();
	
	// TODO: if watcher stalled, do some recovery
	
	distance += robot->odom.getDistanceToGoal();
	
	return distance;
}
