#ifndef AbingtonMaze_h
#define AbingtonMaze_h
#include "Maze.h"

class AbingtonMaze : public Maze {
	public:
		void setup();		
		short getNumNodes();
		short getPathLength();
		short getNumRooms();
};

#endif
