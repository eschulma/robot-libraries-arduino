/*
 * Planner.h
 *
 *  Created on: Feb 20, 2014
 *      Author: ekane
 */

#ifndef PLANNER_H_
#define PLANNER_H_
#include <Arduino.h>
#include "AbingtonMaze.h"

class Planner {
	private:
		boolean bReturningHome;	// must initialize to false
		AbingtonMaze* maze;
		std::list<pathNode>::iterator itMain;

	public:
		mapNode getCurrentNode();
		mapNode chooseNextNode();

		void setCurrentPathIndex(short pathIndex);

		void setup();	// start from beginning, reset iterators etc.
		boolean isFinished();	// done with looking for fire, or returned home
		void returnHome(short startingRoomId);
		boolean isReturningHome() { return bReturningHome; };

		Planner(AbingtonMaze* inMaze) {
			bReturningHome = false;
			maze = inMaze;
		};
};


#endif /* PLANNER_H_ */
