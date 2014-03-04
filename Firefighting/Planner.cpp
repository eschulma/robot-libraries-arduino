/*
 * Planner.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: ekane
 */
#include "Planner.h"
#include "Pilot.h"
#include "AbingtonMaze.h"

#include <StandardCplusplus.h>
#include <serstream>

using namespace std;

ohserialstream serial(Serial);

void Planner::setup() {
	bReturningHome = false;
	itMain = maze->getPathStart();
}

/**
 * Return the map node that is being pointed to through our place on the path
 */
mapNode Planner::getCurrentNode() {
	pathNode p = *itMain;
	mapNode m = maze->getMapNode(p);
	return m;
}

/**
 * Find the target node. If we are at a decision point, we need to determine which path to take.
 */
mapNode Planner::chooseNextNode(Pilot* pilot) {
	if(itMain->decision == NO_DECISION_REQUIRED) {
		itMain++;
	}
	else {
		// TODO: figure out which list we need to splice in
		list<pathNode> branchList; // some maze function
		if(itMain->decision == SOUTHEAST_ROOM_ENTRANCE) {
			if(pilot->isObstaclePresent(MAZE_EAST, 75)) {
				// TODO: add wall to map
				branchList = maze->room3NorthDoorList;
			}
		}
		else {
			serial << F("!!! ERROR Unhandled decision type ") << itMain->decision << F(" for room ") << itMain->id << endl;
		}

		// save our original place
		list<pathNode>::iterator it2 = itMain;

		if(itMain != (maze->pathList).end()) {
			itMain++;
			(maze->pathList).insert(itMain, branchList.begin(), branchList.end());
			itMain = it2;
			itMain++;
		}
		else {
			// we are at the end! move iterator back
			serial << F("WARN: iterator was at the end of the main list") << endl;
			itMain--;
			for(list<pathNode>::iterator it3 = branchList.begin(); it3 != branchList.end(); it3++) {
				(maze->pathList).push_back(*it3);
			}
			itMain++;
		}
	}
	return getCurrentNode();
}

boolean Planner::isFinished() {
}

void Planner::returnHome(short startingRoomId) {
}

/**
 * Because we are potentially splicing in other paths, those may need to be done
 * before we call this; we no longer have one and only one route
 */
void Planner::setCurrentPathIndex(short pathIndex) {
	if(pathIndex < maze->getPathLength()) {
		itMain = maze->getPathStart();
		advance(itMain, pathIndex);
	}
	else {
		serial << F("Error!!! Index for Planner::setCurrentPathIndex is beyond the limit.") << pathIndex << endl;
	}
}
