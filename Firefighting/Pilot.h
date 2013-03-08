#ifndef Pilot_h
#define Pilot_h
#include "Maze.h"
#include "FirefighterRobot.h"

#define PILOT_FIRE_EXTINGUISHED 100

enum pilotFollowMethod {
	PILOT_FOLLOW_LEFT,
	PILOT_FOLLOW_RIGHT,
	PILOT_FOLLOW_LEFT_REAR,
	PILOT_FOLLOW_NONE
};

enum pilotNodeCheck {
	PILOT_CHECK_LEFT,
	PILOT_CHECK_RIGHT,
	PILOT_CHECK_DISTANCE,
	PILOT_CHECK_FORWARD,
	PILOT_CHECK_RIGHT_APPEARANCE
};

class Pilot {
	private:
		short pathIndex;
		mapNode currentNode;
		mapNode nextNode;
		
		mazeHeading heading;
		float distanceToNext;
		Maze* maze;
		FirefighterRobot* robot;
		pilotNodeCheck nodeCheck;
		pilotFollowMethod followMethod;
		long lastPingTime;
		float hallwayWidth;
		
		void changeHeading(mazeHeading currentHeading, mazeHeading newHeading);		
		void nudgeForwardAfterWallLoss(short wallDirection);
		float nudgeToAlign(short wallDirection);
		boolean fightFire();
	public:
		int setCourse();
		int go();		
		void setStart(short startPathIndex, mazeHeading startHeading);
				
		Pilot(Maze* inMaze, FirefighterRobot* inRobot, mazeHeading startHeading);
};

#endif
