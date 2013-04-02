#include "AbingtonMaze.h"
#include <Maze.h>

void AbingtonMaze::setup() {

	nodeRadius = 60;	// cm; need large value to turn sensors on earlier (sigh)
	hallwayWidth = 46;

	// arrays must be allocated and assigned before calling this
	emptyMaze();

	// put in all the nodes and their neighbors, distances, etc.
	nodeList[0].neighbor[MAZE_SOUTH] = 1;
	nodeList[0].distToNeighbor[MAZE_SOUTH] = 77;
	nodeList[0].neighbor[MAZE_EAST] = 5;
	
	nodeList[1].neighbor[MAZE_NORTH] = 0;
	nodeList[1].distToNeighbor[MAZE_NORTH] = nodeList[0].distToNeighbor[MAZE_SOUTH];
	nodeList[1].neighbor[MAZE_WEST] = 2;
	nodeList[1].distToNeighbor[MAZE_WEST] = 72;
	nodeList[1].neighbor[MAZE_SOUTH] = 10; // 8; we can't go from 1 to 8, no good checks
	nodeList[1].distToNeighbor[MAZE_SOUTH] = 120;
	nodeList[1].neighbor[MAZE_EAST] = 4;
	
	nodeList[2].neighbor[MAZE_EAST] = 1;
	nodeList[2].distToNeighbor[MAZE_EAST] = nodeList[1].distToNeighbor[MAZE_WEST];
	nodeList[2].neighbor[MAZE_NORTH] = 3;
	nodeList[2].distToNeighbor[MAZE_NORTH] = 23 + 5;
	
	nodeList[3].isRoom = true;
	nodeList[3].neighbor[MAZE_SOUTH] = 2;
	nodeList[3].distToNeighbor[MAZE_SOUTH] = nodeList[2].distToNeighbor[MAZE_NORTH];
	
	nodeList[4].neighbor[MAZE_WEST] = 1;
	nodeList[4].neighbor[MAZE_NORTH] = 7;
	nodeList[4].neighbor[MAZE_EAST] = 6;
	
	nodeList[5].neighbor[MAZE_WEST] = 0;
	nodeList[5].neighbor[MAZE_SOUTH] = 6;
	
	nodeList[6].neighbor[MAZE_WEST] = 4;
	nodeList[6].neighbor[MAZE_NORTH] = 5;
	
	nodeList[7].isRoom = true;
	nodeList[7].neighbor[MAZE_SOUTH] = 4;
	
	nodeList[8].neighbor[MAZE_NORTH] = 1;
	nodeList[8].neighbor[MAZE_EAST]  = 9;
	nodeList[8].distToNeighbor[MAZE_EAST] = 23 + 5;
	nodeList[8].neighbor[MAZE_SOUTH] = 10;
	
	nodeList[9].isRoom = true;
	nodeList[9].neighbor[MAZE_WEST] = 8;
	
	nodeList[10].neighbor[MAZE_NORTH] = 8;
	nodeList[10].neighbor[MAZE_WEST] = 11;
	nodeList[10].distToNeighbor[MAZE_WEST] = 23 + 5 + 10;
	
	nodeList[11].isRoom = true;
	nodeList[11].neighbor[MAZE_EAST] = 10;
	
	// define path
	pathList[0] = 0;
	pathList[1] = 1;
	pathList[2] = 2;
	pathList[3] = 3;
	pathList[4] = 2;
	pathList[5] = 1;
	pathList[6] = 10;
	pathList[7] = 11;
	pathList[8] = 10;
	pathList[9] = 8;
	pathList[10] = 9;
	pathList[11] = 8;
	pathList[12] = 1;
	pathList[13] = 0;
	pathList[14] = 5;
	pathList[15] = 6;
	pathList[16] = 4;
	pathList[17] = 7;


	// pathList = { 0, 1, 2, 3, 2, 1, 10, 11, 10, 8, 9, 8, 1, 0, 5, 6, 4, 7 };
	
	// define room parameters
	roomList[0].id = 7;
	roomList[0].frontStopDistance = 23;
	roomList[1].id = 3;
	roomList[1].frontStopDistance = 40;
	roomList[2].id = 11;
	roomList[2].frontStopDistance = 40;
	roomList[3].id = 3;
	roomList[3].frontStopDistance = 100;
}
