#ifndef Pilot_h
#define Pilot_h
#include "Maze.h"
#include "DifferentialDriveRobot.h"

#define PILOT_FIRE_EXTINGUISHED 100
#define PILOT_RETURNED_HOME 200

enum pilotFollowMethod {
	PILOT_FOLLOW_LEFT,
	PILOT_FOLLOW_RIGHT,
	PILOT_FOLLOW_NONE
};

enum pilotNodeCheck {
	PILOT_CHECK_LEFT,
	PILOT_CHECK_RIGHT,
	PILOT_CHECK_DISTANCE,
	PILOT_CHECK_FORWARD
//	PILOT_CHECK_RIGHT_APPEARANCE
};

class Pilot {
	private:
		short pathIndex;
		short returnPathIndex;
		mapNode currentNode;
		mapNode nextNode;
		
		mazeHeading heading;
		float distanceToNext;
		Maze* maze;
		DifferentialDriveRobot* robot;
		pilotNodeCheck nodeCheck;
		pilotFollowMethod followMethod;
		float hallwayWidth;
		boolean bSwitchAfterWallCheck;
		boolean bGoingHome;

		long lastPingTime;
		double roomEntryX;
		double roomEntryY;
		double roomEntryHeading;

		void changeHeading(mazeHeading currentHeading, mazeHeading newHeading);		
		void nudgeForwardAfterWallLoss(short wallDirection);
		float nudgeToAlign(short wallDirection);
		boolean fightFire();
	public:
		void setStart(short startPathIndex, mazeHeading startHeading);
		int setCourse();
		int go();		
		int headHome();

		Pilot(Maze* inMaze, DifferentialDriveRobot* inRobot, mazeHeading startHeading);

#ifdef FIREFIGHTER_TEST
		friend class RobotTester;	// for testing
#endif

};

#endif
