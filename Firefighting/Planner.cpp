/*
 * Planner.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: ekane
 */
#include "Planner.h"
#include "AbingtonMaze.h"

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

mapNode Planner::chooseNextNode() {
	// TODO: make sure we are not at the end of the list!

	if(itMain->decision == NO_DECISION_REQUIRED) {
		itMain++;
	}
	else {
		// TODO: figure out which list we need to splice in
		std::list<pathNode> branchList; // some maze function

		itMain++;
		(maze->pathList).insert(itMain, branchList.begin(), branchList.end());
	}
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
		std::advance(itMain, pathIndex);
	}
	else {
		Serial.print("Error!!! Index for Planner::setCurrentPathIndex is beyond the limit.");
		Serial.println(pathIndex);
	}
}
