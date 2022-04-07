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

class MyRobot : public Robot
{
	mutable mutex locking;
	thread th;
	volatile bool is_enabled;
	destination dest;

public:
	MyRobot();
	~MyRobot();	
	AStar::Generator generator;
	deque<path> pathq;
	
	void setDestination(const destination& _dst);
	destination getDestination() const;

	void turning(float angleErr);
	void going(float _fwd, float _rot = 0);
	void route();
	static void generatepath(MyRobot& obj, int _pixels);
	
};

