#include <Arduino.h>
#include "Maze.h"

/**
 * All arrays must be allocated and assigned before calling this!
 */
void Maze::emptyMaze() {
	for(int i = 0; i < getNumNodes(); i++) {
		nodeList[i].id = i;
		nodeList[i].isRoom = false;
		nodeList[i].isVisited = false;
		for(int j = 0; j < 4; j++) {
			nodeList[i].neighbor[j] = MAZE_WALL;
			nodeList[i].distToNeighbor[j] = 0;
		}		
	}
	
	nodeRadius = 23;
	hallwayWidth = 46;
}

float Maze::getHallwayWidth() {
	return hallwayWidth;
}

roomNode Maze::getRoomNode(short id) {
	for(int i = 0; i < getNumRooms(); i++) {
		roomNode room = roomList[i];
		if(room.id == id) {
			return room;
		}
	}
	
	// we didn't find it, but don't want a crash
	Serial.print("Search made for non-existent room node!! id = ");
	Serial.println(id);
	roomNode junk;
	return junk;
}
