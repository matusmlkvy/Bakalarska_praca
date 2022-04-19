#include "MyRobot.h"

MyRobot::MyRobot() : 
    Robot(),
    is_enabled(true),
    astar_th(generatepath, ref(*this), 8),
    route_th(route, ref(*this))
{
    destination dest;
}

MyRobot::~MyRobot()
{
    is_enabled = false;
    astar_th.join();
    route_th.join();
}


void MyRobot::turning(float angleErr)
{
    Wheels_t wh = wheels();

    angleErr -= 360.0 * round(angleErr / 360.0);

    if (angleErr > 30.0) angleErr = 30.0;
    else if (angleErr < -30.0) angleErr = -30.0;

    wh.Left = -2 * angleErr;
    wh.Right = 2 * angleErr;
    setWheels(wh);
}

void MyRobot::going(float _fwd, float _rot)
{
    Wheels_t wh = wheels();

    _rot -= 360.0 * round(_rot / 360.0);

    if (_rot > 45.0) _rot = 45.0;
    else if (_rot < -45.0) _rot = -45.0;

    if (_fwd > 50.0) _fwd = 50.0;
    else if (_fwd < -50.0) _fwd = -50.0;

    wh.Left = 2 * _fwd - 5 * _rot;
    wh.Right = 2 * _fwd + 5 * _rot;
    setWheels(wh);
}

bool turn = false;

void MyRobot::route(MyRobot& obj)
{
    while (obj.is_enabled)
    {
        if (obj.simulationEnabled() || obj.isOpen())
        {


            Position_t pos = obj.position();
            Wheels_t wh = obj.wheels();

            //tolerance in tick
            int distance_tolerance = 40;
            int angle_tolerance = 20;

            path p2;
            bool has_target = false;

            // try to get next target
            {
                lock_guard<mutex> lock(obj.locking);
                if (obj.pathq.size() > 0)
                {
                    p2 = obj.pathq.front();
                    has_target = true;
                }
            }

            if (has_target)
            {
                double go = sqrt(pow((p2.x - pos.x), 2) + pow((p2.y - pos.y), 2));

                double psi_ref = atan2((p2.y - pos.y), (p2.x - pos.x)) * 180 / M_PI;
                double wheels_diff = pos.psi * 0.01 - psi_ref;
                wheels_diff -= round(wheels_diff / 360) * 360;

                if (abs(wheels_diff) > angle_tolerance)
                {
                    turn = true;
                }
                else if (abs(wheels_diff) < 5)
                {
                    turn = false;
                }

                if (turn)
                {
                    obj.turning(wheels_diff);
                }
                else if ((go >= distance_tolerance))
                {
                    obj.going(go, wheels_diff);
                }
                else
                {
                    lock_guard<mutex> lock(obj.locking);
                    if (obj.pathq.size() > 1)
                    {
                        obj.pathq.pop_front();
                        p2 = obj.pathq.front();
                    }
                }
            }
            else
            {
                // nowhere to go
                obj.setWheels(0, 0);
            }
        }

        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

destination MyRobot::getDestination() const
{
    lock_guard<mutex> lock(locking);
    return dest;
}

void MyRobot::setDestination(const destination& _dest)
{
    lock_guard<mutex> lock(locking);
     dest = _dest;
}

vector<path> MyRobot::getroute()
{
    return point_route;
}



void MyRobot::generatepath(MyRobot& obj, int _pixels)
{
    int pixels = _pixels;
    srand(time(NULL));

    
    // Set 2d map size.
    obj.generator.setWorldSize({ 172, 79 });
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    obj.generator.setHeuristic(AStar::Heuristic::euclidean);
    obj.generator.setDiagonalMovement(true);


    while (obj.is_enabled)
    {
        if (obj.isOpen() || obj.simulationEnabled())
        {
            Wheels_t wheels = obj.wheels();
            Position_t pos = obj.position();

            destination _dest = obj.getDestination();

            int posx = (int)std::round((double)pos.x * 1.0 / TICKS_PER_PIXEL / pixels);
            int posy = (int)std::round((double)pos.y * 1.0 / TICKS_PER_PIXEL / pixels);
            int destx = (int)std::round((double)_dest.x * 1.0 / TICKS_PER_PIXEL / pixels);
            int desty = (int)std::round((double)_dest.y * 1.0 / TICKS_PER_PIXEL / pixels);



            if ((posx) != (destx) || (posy) != (desty))
            {
                std::cout << "Generate path ... \n";
                // This method returns vector of coordinates from target to source.
                auto pth = obj.generator.findPath({ posx, posy }, { destx, desty }, (pos.psi / 100));

                path p1;
                {
                    lock_guard<mutex> lock(obj.locking);
                    obj.pathq.clear();
                    obj.point_route.clear();
                    for (auto& coordinate : pth)
                    {
                        p1.x = coordinate.x * TICKS_PER_PIXEL * pixels;
                        p1.y = coordinate.y * TICKS_PER_PIXEL * pixels;
                        obj.pathq.push_front(p1);
                        obj.point_route.push_back(p1);
                    }
                    obj.pathq.pop_front();

                    //route by sa vytvaralo tu v A* a posielal by sa tam kontajner
                }
            }

            {
                lock_guard<mutex> lock(obj.locking);
                bool noPath = false;
                if (obj.pathq.size() <= 1)
                {
                    // nowhere to go
                    noPath = true;
                }
                else
                {
                    auto endPt = obj.pathq.back();
                    if (endPt.x == 0 && endPt.y == 0)
                        noPath = true;
                }

                if (noPath)
                {
                    wheels.Left = 0;
                    wheels.Right = 0;
                    obj.setWheels(wheels);
                    int x = rand() % 170 + 1;
                    obj.dest.x = x * 80;
                    int y = rand() % 77 + 1;
                    obj.dest.y = y * 80;
                    cout << obj.dest.x << "      " << obj.dest.y;
                }
            }
        }

        this_thread::sleep_for(chrono::milliseconds(10));
    }
}
