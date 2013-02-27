#include "AbingtonMaze.h"
#include <Maze.h>

void AbingtonMaze::setup() {

	nodeRadius = 30;	// cm, this is more than half a hallway width (23)
	hallwayWidth = 46;

	mapNode myNodeList[getNumNodes()];
	short myPathList[getPathLength()];
	roomNode myRoomList[getNumRooms()];

	nodeList = myNodeList;
	pathList = myPathList;
	roomList = myRoomList;

	// arrays must be allocated and assigned before calling this
	emptyMaze();

	// put in all the nodes and their neighbors, distances, etc.
	myNodeList[0].neighbor[MAZE_SOUTH] = 1;
	myNodeList[0].distToNeighbor[MAZE_SOUTH] = 77;
	myNodeList[0].neighbor[MAZE_EAST] = 5;
	
	myNodeList[1].neighbor[MAZE_NORTH] = 0;
	myNodeList[1].distToNeighbor[MAZE_NORTH] = myNodeList[0].distToNeighbor[MAZE_SOUTH];
	myNodeList[1].neighbor[MAZE_WEST] = 2;
	myNodeList[1].distToNeighbor[MAZE_WEST] = 72;
	myNodeList[1].neighbor[MAZE_SOUTH] = 8;
	myNodeList[1].neighbor[MAZE_EAST] = 4;
	
	myNodeList[2].neighbor[MAZE_EAST] = 1;
	myNodeList[2].distToNeighbor[MAZE_EAST] = myNodeList[1].distToNeighbor[MAZE_WEST];
	myNodeList[2].neighbor[MAZE_NORTH] = 3;
	myNodeList[2].distToNeighbor[MAZE_NORTH] = 23 + 5;
	
	myNodeList[3].isRoom = true;
	myNodeList[3].neighbor[MAZE_SOUTH] = 2;
	myNodeList[3].distToNeighbor[MAZE_SOUTH] = myNodeList[2].distToNeighbor[MAZE_NORTH];
	
	myNodeList[4].neighbor[MAZE_WEST] = 1;
	myNodeList[4].neighbor[MAZE_NORTH] = 7;
	myNodeList[4].neighbor[MAZE_EAST] = 6;
	
	myNodeList[5].neighbor[MAZE_WEST] = 0;
	myNodeList[5].neighbor[MAZE_SOUTH] = 6;
	
	myNodeList[6].neighbor[MAZE_WEST] = 4;
	myNodeList[6].neighbor[MAZE_NORTH] = 5;
	
	myNodeList[7].isRoom = true;
	myNodeList[7].neighbor[MAZE_SOUTH] = 4;
	
	myNodeList[8].neighbor[MAZE_NORTH] = 1;
	myNodeList[8].neighbor[MAZE_EAST]  = 9;
	myNodeList[8].distToNeighbor[MAZE_EAST] = 23 + 5;
	myNodeList[8].neighbor[MAZE_SOUTH] = 10;
	
	// myNodeList[9].isRoom = true;
	myNodeList[9].neighbor[MAZE_WEST] = 8;
	
	myNodeList[10].neighbor[MAZE_NORTH] = 8;
	myNodeList[10].neighbor[MAZE_WEST] = 11;
	myNodeList[10].distToNeighbor[MAZE_WEST] = 23 + 5 + 10;
	
	// myNodeList[11].isRoom = true;
	myNodeList[11].neighbor[MAZE_EAST] = 10;
	
	// define path
	myPathList[0] = 0;
	myPathList[1] = 1;
	myPathList[2] = 2;
	myPathList[3] = 3;
	
	// define room parameters
	myRoomList[0].id = 7;
	myRoomList[0].frontStopDistance = 23;
	myRoomList[1].id = 3;
	myRoomList[1].frontStopDistance = 40;
}

short AbingtonMaze::getNumNodes() { return 12; }
short AbingtonMaze::getPathLength() { return 4; }
short AbingtonMaze::getNumRooms() { return 2; }
