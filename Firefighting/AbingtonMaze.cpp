#include "AbingtonMaze.h"
#include <Maze.h>

void AbingtonMaze::setup() {

	nodeRadius = 60;// cm; need large value to turn sensors on earlier (sigh)
	hallwayWidth = 46;

	// must be greater than number of nodes!!
	nodeList.resize(12);

	// arrays must be allocated and assigned before calling this
	emptyMaze();

	// put in all the nodes and their neighbors, distances, etc.
	nodeList[0].neighbor[MAZE_SOUTH] = 1;
	nodeList[0].distToNeighbor[MAZE_SOUTH] = 77;
	nodeList[0].neighbor[MAZE_EAST] = 5;

	nodeList[1].neighbor[MAZE_NORTH] = 0;
	nodeList[1].distToNeighbor[MAZE_NORTH] =
			nodeList[0].distToNeighbor[MAZE_SOUTH];
	nodeList[1].neighbor[MAZE_WEST] = 2;
	nodeList[1].distToNeighbor[MAZE_WEST] = 72;
	nodeList[1].neighbor[MAZE_SOUTH] = 10; // 8; we can't go from 1 to 8, no good checks
	nodeList[1].distToNeighbor[MAZE_SOUTH] = 120;
	nodeList[1].neighbor[MAZE_EAST] = 4;
	nodeList[1].distToNeighbor[MAZE_EAST] =
			nodeList[4].distToNeighbor[MAZE_WEST];

	nodeList[2].neighbor[MAZE_EAST] = 1;
	nodeList[2].distToNeighbor[MAZE_EAST] =
			nodeList[1].distToNeighbor[MAZE_WEST];
	nodeList[2].neighbor[MAZE_NORTH] = 3;
	nodeList[2].distToNeighbor[MAZE_NORTH] = 46 + 5;

	nodeList[3].isRoom = true;
	nodeList[3].neighbor[MAZE_SOUTH] = 2;
	nodeList[3].distToNeighbor[MAZE_SOUTH] =
			nodeList[2].distToNeighbor[MAZE_NORTH];

	nodeList[4].neighbor[MAZE_WEST] = 1;
	nodeList[4].distToNeighbor[MAZE_WEST] = 46;	// hallway is wider here, but going short is OK
	nodeList[4].neighbor[MAZE_NORTH] = 7;
	nodeList[4].neighbor[MAZE_EAST] = 6;
	nodeList[4].distToNeighbor[MAZE_NORTH] = 46;

	nodeList[5].neighbor[MAZE_WEST] = 0;
	nodeList[5].neighbor[MAZE_SOUTH] = 6;

	nodeList[6].neighbor[MAZE_WEST] = 4;
	nodeList[6].neighbor[MAZE_NORTH] = 5;

	nodeList[7].isRoom = true;
	nodeList[7].neighbor[MAZE_SOUTH] = 4;
	nodeList[7].distToNeighbor[MAZE_SOUTH] =
			nodeList[4].distToNeighbor[MAZE_NORTH];

	nodeList[8].neighbor[MAZE_NORTH] = 1;
	nodeList[8].neighbor[MAZE_EAST] = 9;
	nodeList[8].distToNeighbor[MAZE_EAST] = 46 + 5 + 20;
	nodeList[8].neighbor[MAZE_SOUTH] = 10;

	nodeList[9].isRoom = true;
	nodeList[9].neighbor[MAZE_WEST] = 8;
	nodeList[9].distToNeighbor[MAZE_WEST] =
			nodeList[8].distToNeighbor[MAZE_EAST];

	nodeList[10].neighbor[MAZE_NORTH] = 8;
	nodeList[10].neighbor[MAZE_WEST] = 11;
	nodeList[10].distToNeighbor[MAZE_WEST] = 46 + 5 + 5;

	nodeList[11].isRoom = true;
	nodeList[11].neighbor[MAZE_EAST] = 10;
	nodeList[11].distToNeighbor[MAZE_EAST] =
			nodeList[10].distToNeighbor[MAZE_WEST];
	// If we add more nodes, change resize command above!!!

	// define path
	pathList.resize(18);	// this is optional

	pathNode p;
	pathList.push_back((pathNode){0, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){1, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){2, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){3, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){2, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){1, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){10, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){11, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){10, SOUTHEAST_ROOM_ENTRANCE});
	pathList.push_back((pathNode){8, NO_DECISION_REQUIRED});
	pathList.push_back((pathNode){9, NO_DECISION_REQUIRED});

	// old pathList = { 0, 1, 2, 3, 2, 1, 10, 11, 10, 8, 9, 8, 1, 0, 5, 6, 4, 7 };

	// define room parameters
	roomList.resize(4);  // optional
	roomList.push_back((roomNode){7, 23});
	roomList.push_back((roomNode){3, 40});
	roomList.push_back((roomNode){11, 40});
	roomList.push_back((roomNode){9, 100});

	returnPathList[0] = 3;	// room
	returnPathList[1] = 2;
	returnPathList[2] = 1;
	returnPathList[3] = 0;
	returnPathList[4] = 7;	// room
	returnPathList[5] = 4;
	returnPathList[6] = 1;
	returnPathList[7] = 0;
	returnPathList[8] = 11;	// room
	returnPathList[9] = 10;
	returnPathList[10] = 8;
	returnPathList[11] = 1;
	returnPathList[12] = 0;
	returnPathList[13] = 9;	// room
	returnPathList[14] = 8;
	returnPathList[15] = 1;
	returnPathList[16] = 0;
}

mapNode AbingtonMaze::getMapNode(short pathIndex) {
	if (pathIndex > getPathLength()) {
		Serial.print("!!!!!!!!!! Error!!! Path index outside bounds: ");
		Serial.println(pathIndex);
		return nodeList[0];
	}

	// get iterator and move it forward
	std::list<pathNode>::iterator it = pathList.begin();
	std::advance(it, pathIndex);
	short nodeIndex = it->id;
	if (nodeIndex > getNumNodes()) {
		Serial.print(
				"!!!!!!!!!! Error!!! Node index for path list outside bounds, index was: ");
		Serial.println(pathIndex);
		return nodeList[0];
	}

	return nodeList[nodeIndex];
}

mapNode AbingtonMaze::getMapNode(pathNode pNode) {
	short nodeIndex = pNode.id;
	if (nodeIndex > getNumNodes()) {
		Serial.print(
				"!!!!!!!!!! Error!!! Node id for path list outside bounds, id was: ");
		Serial.println(pNode.id);
		return nodeList[0];
	}

	return nodeList[nodeIndex];
}

mapNode AbingtonMaze::getReturnMapNode(short returnPathIndex) {
	if (returnPathIndex > getReturnPathLength()) {
		Serial.print("!!!!!!!!!! Error!!! Return path index outside bounds: ");
		Serial.println(returnPathIndex);
		return nodeList[0];
	}

	short nodeIndex = returnPathList[returnPathIndex];
	if (nodeIndex > getNumNodes()) {
		Serial.print(
				"!!!!!!!!!! Error!!! Node index for return path list outside bounds, index was: ");
		Serial.println(returnPathIndex);
		return nodeList[0];
	}

	return nodeList[nodeIndex];
}

short AbingtonMaze::getPathLength() {
	return pathList.size();
}

 short AbingtonMaze::getReturnPathLength() {
	return MAZE_RETURN_PATH_LENGTH;
}
