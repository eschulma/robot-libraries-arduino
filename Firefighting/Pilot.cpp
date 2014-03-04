#include <Pilot.h>

#define DEBUG_DELAY 0
#define PRE_ALIGN_DELAY 100
#define TURN_DELAY 100

#include <StandardCplusplus.h>
#include <serstream>

using namespace std;

extern ohserialstream serial;

Pilot::Pilot(Maze* inMaze, Planner* inPlanner, FireCheetah* inRobot, mazeHeading startHeading){
	maze = inMaze;
	planner = inPlanner;
	robot = inRobot;
	heading = startHeading;
	lastPingTime = 0;

	followMethod = PILOT_FOLLOW_NONE;
	nodeCheck = PILOT_CHECK_FORWARD;
	bSwitchAfterWallCheck = false;
	bGoingHome = false;

	planner->setup();
}

void Pilot::setStart(short startPathIndex, mazeHeading startHeading) {
	planner->setCurrentPathIndex(startPathIndex);
	heading = startHeading;
}

boolean Pilot::fightFire() {
	serial << F("Checking for fire!") << endl;
	// save coords so we can return later
	roomEntryX = robot->getX();
	roomEntryY = robot->getY();
	roomEntryHeading = robot->getHeading();

	serial << F("Room entry coordinates: ") << roomEntryX << F(", ") << roomEntryY << F(", ") << roomEntryHeading << endl;

	int degrees = robot->panServoForFire();
	if(degrees != ROBOT_NO_FIRE_FOUND) {
		robot->fightFire(degrees);
		robot->stop();
		return true;
	}
	return false;
}

/**
 *	Figure out where we are going next.
 *
 **/
int Pilot::setCourse() {
	currentNode = planner->getCurrentNode();
	if(bGoingHome) {
		serial << F("Heading home.") << endl;
	}
	else {
		serial << F("Looking for candle.") << endl;
	}

	if((bGoingHome) && (currentNode.id == 0)) {
		return PILOT_RETURNED_HOME;
	}

	// we have arrived in a room -- time to look for the candle, continue on, or go home
    if(currentNode.isRoom) {
    	if(!bGoingHome) {
			if(fightFire()) {
				return PILOT_FIRE_EXTINGUISHED;
			}
    	}
    	else {
    		Serial.println("Beginning trip home.");

    		// may be close to a wall -- check
    		float wallDistance = robot->getSideWallDistance(ROBOT_LEFT);
    		if((wallDistance != ROBOT_NO_VALID_DATA) && (wallDistance < 10.0)) {
    			robot->turn(15.0 * DEG_TO_RAD);
    			robot->stop();
    			delay(TURN_DELAY);
    			robot->backUp(10.0);
    			robot->stop();
        		// for backing up, we should allow extra time before making motors go forward again
        		delay(2 * TURN_DELAY);
    		}
    		else {
        		wallDistance = robot->getSideWallDistance(ROBOT_RIGHT);
        		if((wallDistance != ROBOT_NO_VALID_DATA) && (wallDistance < 10.0)) {
        			robot->turn(-15.0 * DEG_TO_RAD);
        			robot->stop();
        			delay(TURN_DELAY);
        			robot->backUp(10.0);
        			robot->stop();
            		// for backing up, we should allow extra time before making motors go forward again
            		delay(2 * TURN_DELAY);
        		}
    		}

    		// move to the entry spot
    		robot->goToGoal(roomEntryX, roomEntryY);
    		robot->stop();
       		delay(TURN_DELAY);

    		// turn so we are facing 180 from where we entered
    		double desiredHeading = roomEntryHeading + PI;
    		desiredHeading = atan2(sin(desiredHeading), cos(desiredHeading));
    		double currentHeading = robot->getHeading();
    		robot->turn(desiredHeading - currentHeading);
    		robot->stop();
    		delay(TURN_DELAY);

    		// update heading appropriately
    		heading = (mazeHeading)((heading + 2) % 4);
    	}
	}

    if(planner->isFinished()) {
    	if(!bGoingHome) {
			// we are at the last node..and failed to find the fire, presumably.
			// we stop at second to last path index because setCourse includes the next node
    		serial << F("Finished course, no joy.") << endl;
			return -1;
		}
    	else {
    		serial << F("!! Exceeded path limits on return home.") << endl;
			return -1;
		}
    }

    nextNode = planner->chooseNextNode(this);

    /*
	Serial.println("");
	if(!bGoingHome) {
		Serial.print("At path index ");
		Serial.println(pathIndex);
	}
	else {
		Serial.print("At return path index ");
		Serial.println(returnPathIndex);
	} */

    serial << F("Current node id: ") << currentNode.id << F("Next node id: ") << nextNode.id << endl;
	
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
		serial << F("Can't find neighbor path for current node ") << currentNode.id << endl;
		return -2;
	}
	
	// we can't set hallway width in the constructor, as maze is not ready yet
	hallwayWidth = maze->getHallwayWidth();
	if(hallwayWidth == 0) {
		serial << F("Hallway width was zero!!! Corrupted memory, exiting.") << endl;
		return -2;
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
		
	// how do we check we've gotten there? Handtuned items first.
	if((currentNode.id == 6) && (nextNode.id == 4)) {
		nodeCheck = PILOT_CHECK_RIGHT;
		followMethod = PILOT_FOLLOW_LEFT;
		doAlignLeft = true;
	}
	else if((currentNode.id == 1) && (nextNode.id == 10)) {
		// hand-tune for 1 to 10, otherwise it will try to align with a non-existent left wall!
		nodeCheck = PILOT_CHECK_RIGHT;
		followMethod = PILOT_FOLLOW_RIGHT;
	}
	else if((currentNode.id == 9) && (nextNode.id == 8)) {
		nodeCheck = PILOT_CHECK_RIGHT;
		followMethod = PILOT_FOLLOW_RIGHT;
	}
	else if( nextNode.neighbor[ newHeading ] == MAZE_WALL ) {
		// forward wall detection is safest
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
	else {
		// this shouldn't happen; unless we add wall appearance as a node option
		serial << F("Can't find right check to use for next node, current node is ") << currentNode.id << endl;
		return -2;
	}
				
	// update current node to new one
	currentNode.isVisited = true;
	distanceToNext = nodeDistance;
	
	// for rooms, follow left wall in
	if(nextNode.isRoom) {
		serial << F("Next node is a room.") << endl;
		nodeCheck = PILOT_CHECK_DISTANCE;
		followMethod = PILOT_FOLLOW_LEFT;

		if(nextNode.id == 7) {
			followMethod = PILOT_FOLLOW_NONE;
		}

		if(distanceToNext == 0) {
			distanceToNext = hallwayWidth + 5;
		}
	}
	
	boolean useSonarAlignment = true;
	if(useSonarAlignment) {
		serial << F("Using sonar alignment.") << endl;
		if(followMethod == PILOT_FOLLOW_RIGHT) {
			doAlignRight = true;
		}
		else if(followMethod == PILOT_FOLLOW_LEFT) {
			doAlignLeft = true;
		}
	}

	// (put any handtuning here)
	/* if(pathIndex == 0) {
		doAlignLeft = false;
		doAlignRight = false;
	} */
	if((currentNode.id == 1) && (nextNode.id == 2)) {
		followMethod = PILOT_FOLLOW_LEFT;
		doAlignLeft = true;
	}	
	if((currentNode.id == 1) && (nextNode.id == 8)) {
		followMethod = PILOT_FOLLOW_RIGHT;
		doAlignRight = true;
	}
	if(currentNode.isRoom) {
		// we are following right wall out, except for room 7
		followMethod = PILOT_FOLLOW_RIGHT;
		if(currentNode.id == 7) {
			followMethod = PILOT_FOLLOW_NONE;
		}

		doAlignRight = true;
	}

	// if we will be following a wall, make sure that it is in view first.
	if(doAlignRight || doAlignLeft) {
		short wallSide = ROBOT_RIGHT;
		if(!doAlignRight) {
			wallSide = ROBOT_LEFT;
		}

		Serial.print("Preparing to align side ");
		Serial.println(wallSide);

		int heading;
		if(wallSide == ROBOT_LEFT ) {
			heading = (newHeading + 3) % 4; // left heading
		}
		else {
			heading = (newHeading + 1) % 4; // right heading
		}

		if(currentNode.neighbor[heading] != MAZE_WALL) {
			serial << F("Moving forward to find wall.") << endl;
						
			float dist = nudgeToAlign(wallSide);
			distanceToNext -= dist;
		}
		
		// And now. Try to align with wall.
		delay(PRE_ALIGN_DELAY);
		if(robot->align(wallSide)) {
			delay(TURN_DELAY);	// delays after turns are important
		}
	}

	// for the island room exit, only follow wall if it is visible
	if((currentNode.id == 4) && (nextNode.id == 1)) {
		if(robot->isSideWallLost(ROBOT_LEFT)) {
			nodeCheck = PILOT_CHECK_DISTANCE;
			followMethod = PILOT_FOLLOW_NONE;
		}
	}

	// if we are using wall checks, switch to forward check when wall lost, if possible
	// only applies to approach for node 10 right now
	bSwitchAfterWallCheck = false;
	if((nextNode.neighbor[heading] == MAZE_WALL) && // (nextNode.id != 8) &&
			((nodeCheck == PILOT_CHECK_LEFT) || (nodeCheck == PILOT_CHECK_RIGHT))) {
		Serial.println("Will use front sonar to go forward after wall loss.");
		bSwitchAfterWallCheck = true;
	}

	// put this at the very end!
	if((followMethod == PILOT_FOLLOW_NONE) || (bSwitchAfterWallCheck)) {
		// set a goal so we drive forward
		robot->setGoal(1000, 0);
	}
		
	robot->resetOdometers();
	robot->resetStallWatcher();
	// done so getDistanceTravelled is accurate:
	robot->markPosition();
	robot->updateOdometry();
	// make sure we start at a reasonable speed
	robot->resetCalculatedMovePWMs();
	
	if(!bGoingHome) {
		serial << F("For node ");
	}
	else {
		serial << F("For return node ");
	}
	serial << currentNode.id << endl;

	serial << F("Follow method is ") << followMethod << endl << F("Check method is ") << nodeCheck << endl;
	serial << F("Distance to next node is ") << distanceToNext << endl;
	// Serial.print("Front stop distance is ");
	// Serial.println(frontStopDistance);
	
	return 0;
}

void Pilot::changeHeading(mazeHeading currentHeading, mazeHeading newHeading) {
	int delta = (newHeading - currentHeading + 4) % 4;
	
	if(delta != 0) {
		serial << F("Changing heading from ") << currentHeading << F(" to ") << newHeading << endl;
		
		robot->stop();
		delay(TURN_DELAY);

		double my90 = 90 * DEG_TO_RAD;
	
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
			robot->turn(2.0 * my90);
		}
		// let motors settle
		robot->stop();
		delay(TURN_DELAY); // important! (was 1500)
	}
	else {
		serial << F("No heading change.") << endl;
	}

	heading = newHeading;
}

int Pilot::go() {	
	if(robot->isStalled()) {
		boolean doRecovery = true;

		// if we are following a wall or moving, are not blocked, and haven't reached full power (use fabs), keep going
		if(!robot->isWayForwardBlocked()) {
			if((followMethod == PILOT_FOLLOW_LEFT) || (followMethod == PILOT_FOLLOW_RIGHT)) {
				if(fabs(robot->getFollowWallCalculatedPWM()) < robot->getMaxAllowedPWM()) {
					doRecovery = false;
				}
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
				// re-orient goal point after turn
				robot->setGoal(distanceToNext + maze->getNodeRadius() + 50, 0);
			}
		}
	}

	// We must make the distance trigger far enough that we do not simply follow the
	// wall around the turn!
	float bufferDistance = 17;
	const float desiredWallDistance = ((hallwayWidth - robot->getTrackWidth()) / 2.0) + 2;
	
	robot->updateOdometry();
	float distanceTravelled = robot->getDistanceFromMarkedPoint();
	
	// straight distance check first
	if(nodeCheck == PILOT_CHECK_DISTANCE) {
		if((distanceTravelled + 3.0) > distanceToNext) {
				serial << F("PILOT_CHECK_DISTANCE triggered.") << endl;
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
						serial << F("PILOT_CHECK_FORWARD triggered.") << endl;
						delay(PRE_ALIGN_DELAY * 1.5);	// don't remove this! Can mess up alignment otherwise

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

						// if re-aligned, do one last check...
						if(bAlignDone) {
							wallDistance = robot->getFrontWallDistance();
							if(wallDistance > desiredWallDistance + bufferDistance) {
								serial << F("Alignment gives cancellation of front wall check.") << endl;
								return 0;
							}
							else {
								delay(TURN_DELAY);
							}
						}
							
						serial << F("Front wall distance ") << wallDistance << endl;
//						Serial.print("Desired walldistance ");
//						Serial.println(desiredWallDistance);

						// don't move for small things
						if(fabs(wallDistance - desiredWallDistance) < 5) {
							wallDistance = desiredWallDistance;
						}
						
						serial << F("Moving ") << (wallDistance - desiredWallDistance) << endl;
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
						serial << F("PILOT_CHECK_LEFT/RIGHT triggered.") << endl;

						// see if we are way off in angle, and if so, try to align
						robot->stop();
						delay(PRE_ALIGN_DELAY);
						robot->resetStallWatcher();

						boolean bTurnDone = false;
						float theta = robot->getMisalignmentAngle(wallSide);
						if((theta != ROBOT_NO_VALID_DATA) && (fabs(theta) > maxMisalignment)) {
							// this should work for both right and left sides
							serial << F("Turning to find wall (degrees): ") << (theta * RAD_TO_DEG) << endl;
							robot->turn(theta);
							robot->stop();

							bTurnDone = true;
						}

						// if no turn, or turned but wall still lost, give up
						if((!bTurnDone) || (robot->isSideWallLost(wallSide))) {
							serial << F("Lost wall: ") << wallSide << endl;

							if(!bSwitchAfterWallCheck) {
								nudgeForwardAfterWallLoss(wallSide);
							}
							else {
								serial << F("Switching to forward check.") << endl;
								followMethod = PILOT_FOLLOW_NONE;
								nodeCheck = PILOT_CHECK_FORWARD;
								return 0;
							}
							return 1;
						}
						else {
							serial << F("Re-found wall.") << endl;

							// try to avoid endless loop
							lastPingTime += 300;
						}
					}
				}
			} // if checking wall still present

//			if(nodeCheck == PILOT_CHECK_RIGHT_APPEARANCE) {
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
		serial << F("Unknown follow method ") << followMethod << endl;
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
	// robot->resetCalculatedMovePWMs();

	float fudgeFactor = 0.0;
	if((currentNode.id == 0) && (nextNode.id == 1)) {
		fudgeFactor = 2.0;
	}

	float nudgeDistance = robot->getCalculatedWallEnd(wallDirection) + (hallwayWidth / 2.0) + fudgeFactor;

	robot->move(nudgeDistance, 0);
	robot->stop();
	if(robot->isStalled()) {
		robot->resetStallWatcher();
		robot->backUp(10);
		robot->stop();
	}

	// did we make it past the wall?
	delay(PRE_ALIGN_DELAY);
	if(robot->isSideWallPresent(wallDirection)) {
		serial << F("Still seeing wall, mudging extra.") << endl;
		robot->move(10, 0);
		robot->stop();
	}
	
	serial << F("Nudged forward ") << nudgeDistance << endl;

	// special for island room
	if((nextNode.id == 4) && (heading == MAZE_WEST)) {
		serial << F("Aligning for island room (7).") << endl;
		delay(PRE_ALIGN_DELAY);
		robot->align(ROBOT_LEFT);
	}
}

/**
 *	Move robot to where both sonar can see the wall it needs to align with. It's very important
 *	to make sure that we are on course here.
 */
float Pilot::nudgeToAlign(short wallDirection) {
	serial << F("nudgeToAlign called.") << endl;

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
	int numRecovers = 0;
	int numProactiveRecovers = 0;

	while(doLoop) {
		if(robot->isStalled()) {
			if((!robot->isWayForwardBlocked()) && (fabs(robot->getMoveCalculatedPWM()) < robot->getMaxAllowedPWM())) {
				robot->resetStallWatcher();
			}
			else {
				if(numRecovers < 5) {
					numRecovers++;
					robot->recover();
					robot->stop();
				}
				else {
					// OK, we have a real problem here
					serial << F("Nudge to align, standard recoveries have failed.") << endl;
					robot->resetStallWatcher();
					int closeSide = robot->getSideClosestToForwardObstacle();
					robot->backUp(10.0);
					robot->stop();
					robot->resetStallWatcher();
					if(closeSide != ROBOT_NO_VALID_DATA) {
						if(closeSide == ROBOT_LEFT) {
							robot->turn(-30.0);
						}
						else {
							robot->turn(30.0);
						}
					}
					robot->resetStallWatcher();
					numRecovers = 0;
				}

				robot->resetStallWatcher();
				// so goal point is re-oriented properly
				robot->setGoal(500, 0);
			}
		}
		else {
			numRecovers = 0;
		}

		// drive slowly forward
		robot->driveTowardGoal(0.5);

		// can't use getSideWallDistance, it tries repeatedly so is too slow
		// (even this version takes 150ms)
		if(robot->getDistanceFromMarkedPoint() > minTravelRequired) {
			if(robot->isAlignmentPossible(wallDirection, hallwayWidth * 1.4)) {
				serial << F("Alignment possible.") << endl;
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
				serial << F("nudgeToAlign has encountered a forward obstacle") << endl;
				delay(500);

				if(numProactiveRecovers < 5) {
					angleTurned += robot->recover();
					serial << F("Recovery turned: ") << angleTurned << endl;

					// so goal point is re-oriented properly
					robot->setGoal(500, 0);

					// we may have been on an outside corner..in which case, should
					// move forward a while before testing alignment again
					minTravelRequired = robot->getDistanceFromMarkedPoint() +
							(minAllowedFrontWallDistance * 1.4 * 1.3);

					lastPingTime += 100;
				}
				else {
					serial << F("Nudge to align ignoring forward obstacle warnings.") << endl;
				}
			} // if forward wall detected
			else {
				numProactiveRecovers = 0;
			}
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

int Pilot::headHome() {
	serial << F("In head home.") << endl;
	short roomId = currentNode.id;

	planner->returnHome(roomId);

	// uh-oh, didn't find it
	serial << F("!!! Unable to find return path index for node id ") << roomId << endl;
	return -1;
}

/*
 * See if there is an obstacle present in the given direction (heading)
 * less than maxDistance away. We align first if possible, then use the appropriate
 * sensor suite, without turning unless needed).
 */
boolean Pilot::isObstaclePresent(mazeHeading targetHeading, float maxDistance) {
	serial << F("In isObstaclePresent for target heading ") << targetHeading << F(", maxDistance ") << maxDistance << endl;
	boolean bReturn = true;

	int delta = (targetHeading - heading + 4) % 4;

	if(delta == 2) {
		// it's in back of us, damn; turn so it's on the side
		mazeHeading newHeading = (mazeHeading)((heading + 1) % 4);
		changeHeading(heading, newHeading);

		delta = (targetHeading - heading + 4) % 4;
	}

	// do alignment if possible, but not on the side we are looking for
	mazeHeading wallHeading = (mazeHeading)((heading + 1) % 4);
	if((wallHeading != targetHeading) && (currentNode.neighbor[wallHeading] == MAZE_WALL)) {
		robot->align(ROBOT_RIGHT);
	}
	else {
		wallHeading = (mazeHeading)((heading + 2) % 4);
		if((wallHeading != targetHeading) && (currentNode.neighbor[wallHeading] == MAZE_WALL)) {
			robot->align(ROBOT_LEFT);
		}
		else {
			serial << F("No wall to align robot.") << endl;
		}
	}

	float distance = 0;
	if(delta == 0) {
		// we are facing the right way
		distance = robot->getFrontWallDistance();
	}
	else if(delta == 1) {
		// it is on the right
		distance = robot->getSideWallDistance(ROBOT_RIGHT);
	}
	else if(delta == 3) {
		// it is on the left
		distance = robot->getSideWallDistance(ROBOT_LEFT);
	}

	if((distance == NO_SONAR_FRONT_WALL_SEEN) || (distance == ROBOT_NO_VALID_DATA) || (distance > maxDistance)) {
		bReturn = false;
	}

	serial << F("Obstacle detected: ") << bReturn << endl;

	return bReturn;
}
