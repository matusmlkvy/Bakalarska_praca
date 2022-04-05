#include "MyRobot.h"


void MyRobot::turning(EPuck::Robot& robo, float angleErr)
{
    Wheels_t wheels = robo.wheels();

    angleErr -= 360.0 * round(angleErr / 360.0);

    if (angleErr > 30.0) angleErr = 30.0;
    else if (angleErr < -30.0) angleErr = -30.0;

    wheels.Left = -2 * angleErr;
    wheels.Right = 2 * angleErr;
    robo.setWheels(wheels);
}

void MyRobot::going(EPuck::Robot& robo, float _fwd, float _rot)
{
    Wheels_t wheels = robo.wheels();

    _rot -= 360.0 * round(_rot / 360.0);

    if (_rot > 45.0) _rot = 45.0;
    else if (_rot < -45.0) _rot = -45.0;

    if (_fwd > 50.0) _fwd = 50.0;
    else if (_fwd < -50.0) _fwd = -50.0;

    wheels.Left = 2 * _fwd - 5 * _rot;
    wheels.Right = 2 * _fwd + 5 * _rot;
    robo.setWheels(wheels);
}

bool turn = false;

void MyRobot::route(EPuck::Robot & robo, deque<path> _pathq)
{
    Position_t pos = robo.position();
    Wheels_t wheels = robo.wheels();

    //tolerance in tick
    int distance_tolerance = 40;
    int angle_tolerance = 10;

    path p2;
    {
        lock_guard<mutex> lock(locking);
        if (_pathq.size() > 0)
            p2 = _pathq.front();
        else
            return;
    }

    double go = sqrt(pow((p2.x - pos.x), 2) + pow((p2.y - pos.y), 2));

    double psi_ref = atan2((p2.y - pos.y), (p2.x - pos.x)) * 180 / M_PI;
    double wheels_diff = pos.psi * 0.01 - psi_ref;
    wheels_diff -= round(wheels_diff / 360) * 360;

    if (abs(wheels_diff) > angle_tolerance)
    {
        turning(robo, wheels_diff);
        turn = true;
    }
    else if (abs(wheels_diff) > 1 && abs(wheels_diff) <= angle_tolerance && turn == true)
    {
        turning(robo, wheels_diff);
        if (abs(wheels_diff) < 2)
        {
            turn = false;
        }
    }
    else if ((go >= distance_tolerance))
    {
        going(robo, go, wheels_diff);
    }
    else
    {
        lock_guard<mutex> lock(locking);
        if (_pathq.size() > 1)
        {
            _pathq.pop_front();
            p2 = _pathq.front();
        }
        else
            return;
    }
}


vector<path> help;
void MyRobot::generatepath(EPuck::Robot& robo, destination _dest, int _pixels)
{
    int pixels = _pixels;
    srand(time(NULL));

    deque<path> pathq;

    
    // Set 2d map size.
    generator.setWorldSize({ 172, 79 });
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);


    while (true)
    {
        Wheels_t wheels = robo.wheels();
        Position_t pos = robo.position();

        int posx = (int)std::round((double)pos.x * 1.0 / TICKS_PER_PIXEL / pixels);
        int posy = (int)std::round((double)pos.y * 1.0 / TICKS_PER_PIXEL / pixels);
        int destx = (int)std::round((double)_dest.x * 1.0 / TICKS_PER_PIXEL / pixels);
        int desty = (int)std::round((double)_dest.y * 1.0 / TICKS_PER_PIXEL / pixels);



        if ((posx) != (destx) || (posy) != (desty))
        {
            std::cout << "Generate path ... \n";
            // This method returns vector of coordinates from target to source.
            auto pth = generator.findPath({ posx, posy }, { destx, desty }, (pos.psi / 100));

            path p1;
            {
                lock_guard<mutex> lock(locking);
                help.clear();
                pathq.clear();
                for (auto& coordinate : pth)
                {
                    p1.x = coordinate.x * TICKS_PER_PIXEL * pixels;
                    p1.y = coordinate.y * TICKS_PER_PIXEL * pixels;
                    help.push_back(p1);
                    pathq.push_front(p1);
                }
                pathq.pop_front();

                //route by sa vytvaralo tu v A* a posielal by sa tam kontajner
            }
        }

        {
            lock_guard<mutex> lock(locking);
            bool noPath = false;
            if (pathq.size() <= 1)
            {
                // nowhere to go
                noPath = true;
            }
            else
            {
                auto endPt = pathq.back();
                if (endPt.x == 0 && endPt.y == 0)
                    noPath = true;
            }

            if (noPath)
            {
                wheels.Left = 0;
                wheels.Right = 0;
                robo.setWheels(wheels);
                int x = rand() % 170 + 1;
                _dest.x = x * 80;
                int y = rand() % 77 + 1;
                _dest.y = y * 80;
                cout << _dest.x << "      " << _dest.y;
            }
        }
        route(robo, pathq);
    }
}