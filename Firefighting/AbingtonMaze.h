#ifndef AbingtonMaze_h
#define AbingtonMaze_h
#include "Maze.h"

enum decisionType {
	NO_DECISION_REQUIRED,
	SOUTHEAST_ROOM_ENTRANCE,
	DOG_AREA
};

typedef struct {
	short id;
	decisionType decision;
} pathNode;

class AbingtonMaze : public Maze {
	public:
		void setup();
		short getPathLength();
		short getReturnPathLength();

		std::list<pathNode>::iterator getPathStart() { return pathList.begin(); };

		mapNode getMapNode(short pathIndex);
		mapNode getMapNode(pathNode pNode);
		mapNode getReturnMapNode(short pathIndex);

	protected:
		std::list<pathNode> pathList;
		short returnPathList[MAZE_RETURN_PATH_LENGTH];

	friend class Planner;
};

#endif
