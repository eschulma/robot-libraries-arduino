#ifndef Maze_h
#define Maze_h
#include <Arduino.h>

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
		mapNode* nodeList;
		short* pathList;
		roomNode* roomList;
	
		float nodeRadius;
		float hallwayWidth;
		void emptyMaze();	// be careful, arrays must be allocated and assigned abefore calling this
	public:
		virtual short getNumNodes();
		virtual short getNumRooms();
		virtual short getPathLength();

		boolean isRoom(short nodeIndex) { return nodeList[nodeIndex].isRoom; };
		float getNodeRadius() { return nodeRadius; };
		float getHallwayWidth();
		mapNode getPathNode(int index) { return nodeList[pathList[index]]; };
		roomNode getRoomNode(short id);
		
		Maze() {};
};

#endif
