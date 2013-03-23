#ifndef Maze_h
#define Maze_h
#include <Arduino.h>

#define MAZE_NUM_NODES 12
#define MAZE_PATH_LENGTH 6
#define MAZE_NUM_ROOMS 2

typedef struct {
	short id;
	short neighbor[4];
	float distToNeighbor[4];
	boolean isRoom;	
	boolean isVisited;
} mapNode;

typedef struct {
	short id;
	float frontStopDistance;
} roomNode;
	

enum mazeHeading {
	MAZE_NORTH = 0,
	MAZE_EAST = 1,
	MAZE_SOUTH= 2,
	MAZE_WEST = 3
};

#define MAZE_WALL -1
#define MAZE_OPEN_SPACE -2

class Maze {
	private:
		virtual void setup() = 0;
	protected:
		mapNode nodeList[MAZE_NUM_NODES];
		short pathList[MAZE_PATH_LENGTH];
		roomNode roomList[MAZE_NUM_ROOMS];
	
		float nodeRadius;
		float hallwayWidth;
		void emptyMaze();	// be careful, arrays must be allocated and assigned abefore calling this
	public:
		short getNumNodes() { return MAZE_NUM_NODES; };
		short getNumRooms() { return MAZE_NUM_ROOMS; };
		short getPathLength() { return MAZE_PATH_LENGTH; };

		mapNode getPathNode(short pathIndex);
		boolean isRoom(short nodeIndex) { return nodeList[nodeIndex].isRoom; };
		float getNodeRadius() { return nodeRadius; };
		float getHallwayWidth();
		roomNode getRoomNode(short id);
		
		Maze() {};
};

#endif
