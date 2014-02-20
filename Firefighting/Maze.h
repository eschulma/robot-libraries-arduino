#ifndef Maze_h
#define Maze_h
#include <Arduino.h>

// STL -- see http://andybrown.me.uk/wk/2011/01/15/the-standard-template-library-stl-for-avr-with-c-streams/
#include <new.h>	// Arduino library that is needed for STL to work
#include <iterator>
#include <vector>
#include <list>

// #define MAZE_NUM_NODES 12
// #define MAZE_PATH_LENGTH 18
// #define MAZE_NUM_ROOMS 4
#define MAZE_RETURN_PATH_LENGTH 17


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
		// children must provide these
		virtual void setup() = 0;
	protected:
		std::vector<mapNode> nodeList;
		std::vector<roomNode> roomList;

		float nodeRadius;
		float hallwayWidth;
		void emptyMaze();	// be careful, arrays must be allocated and assigned abefore calling this
	public:
		virtual short getPathLength() = 0;
		virtual short getReturnPathLength() = 0;
		virtual mapNode getPathNode(short pathIndex) = 0;
		virtual mapNode getReturnPathNode(short pathIndex) = 0;

		short getNumNodes() { return nodeList.size(); };
		short getNumRooms() { return roomList.size(); };

		boolean isRoom(short nodeIndex) { return nodeList[nodeIndex].isRoom; };
		float getNodeRadius() { return nodeRadius; };
		float getHallwayWidth();
		roomNode getRoomNode(short id);
		
		Maze() {};
};

#endif
