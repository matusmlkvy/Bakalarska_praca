#pragma once
#include <thread>
#include <stdlib.h>
#include <time.h>
#include <queue>
#include <cstring>
#include <iostream>

#include <OccupancyGrid.h>
#include <defs.h>
#include <Robot.h>
#include "Astar.h"

using namespace EPuck;
using namespace std;
using namespace placeholders;

#define TICKS_PER_PIXEL 10

struct destination
{
	int x = 8000;
	int y = 4000;
};
struct path
{
	double x;
	double y;
};

class MyRobot
{
	mutex locking;
public:
	MyRobot();
	~MyRobot();
	Robot robo;
	AStar::Generator generator;
	deque<path> pathq;
	void turning(EPuck::Robot& robo, float angleErr);
	void going(EPuck::Robot& robo, float _fwd, float _rot = 0);
	void route(EPuck::Robot& robo, deque<path> _pathq);
	void generatepath(EPuck::Robot& robo, destination _dest, int _pixels);
	
};

