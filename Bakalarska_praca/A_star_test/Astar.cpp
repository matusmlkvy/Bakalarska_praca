#include "Astar.h"
#include <algorithm>

using namespace std::placeholders;


bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node* parent_, int dir_)
{
    parent = parent_;
    coordinates = coordinates_;
    dir = dir_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
    //zadefinovat uhly pomocou 45,90 ....
    directionAngles = {90, 0, -90, -180, -135, 45, 135, -45};
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}


void AStar::Generator::addCollision(Vec2i coordinates_)
{
    std::lock_guard<std::mutex> locker(lock_walls);
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    std::lock_guard<std::mutex> locker(lock_walls);
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end())
    {
        walls.erase(it);
    }
}

void AStar::Generator::addrobot(Vec2i coordinates_)
{
    std::lock_guard<std::mutex> locker(lock_walls);
    robots.push_back(coordinates_);
}

void AStar::Generator::clearrobot()
{
    std::lock_guard<std::mutex> locker(lock_walls);
    robots.clear();
}

//dorobit moving obstacles nieco podobne ako walls len s premenlivymi prekazkami a ked najde cestu premaze


void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_, int dir_) 
{
    Node* current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(500);
    closedSet.reserve(500);
    openSet.push_back(new Node(source_, nullptr, dir_));

    while (!openSet.empty())
    {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++)
        {
            auto node = *it;
            if (node->getScore() <= current->getScore())
            {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_)
        {
            break;
        }


        //zadefinovat niekam prerusenie po to com sa nedokazem dostat do ciela, opytat sa 

        closedSet.push_back(current);
        openSet.erase(current_it);

        if (closedSet.size() > 20000)
        {
            current = new Node(source_, nullptr, dir_);
            closedSet.push_back(current);
            break;
        }

        for (uint i = 0; i < directions; ++i)
        {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            int newDir = directionAngles[i];
            int danger = detectCollision(newCoordinates);
            int danger_robot = detectRobot(newCoordinates);
            if (findNodeOnList(closedSet, newCoordinates) || danger > 400 || danger_robot > 400)
            {
                continue;
            }
            
            int dirDiff = newDir - current->dir;
            dirDiff -= (int)(round((double)dirDiff / 360.0) * 360.0);
            uint totalCost = current->G + ((i < 4) ? 10 : 14) + abs(dirDiff) + danger + danger_robot;

            Node* successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr)
            {
                successor = new Node(newCoordinates, current, newDir);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G)
            {
                successor->parent = current;
                successor->G = totalCost;
                successor->dir = newDir;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr)
    {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_)
    {
        if (node->coordinates == coordinates_)
        {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();)
    {
        delete* it;
        it = nodes_.erase(it);
    }
}

int AStar::Generator::detectCollision(Vec2i coordinates_)
{
    std::lock_guard<std::mutex> locker(lock_walls);
    int dmin = 10000;
    int xc = coordinates_.x;
    int yc = coordinates_.y;
    for (int i = 0; i < walls.size(); i++)
    {
        int xw = walls[i].x;
        int yw = walls[i].y;
        int dx = xw - xc;
        int dy = yw - yc;
        int d = dx * dx + dy * dy;
        if (d < dmin)
        {
            dmin = d;
        }
    }
    if (dmin == 0)
    {
        return 10000;
    }
    else if (dmin < 40)
    {
        return 1000 / dmin;
    }
    else
    {
        return 0;
    }
}

int AStar::Generator::detectRobot(Vec2i coordinates_)
{
    std::lock_guard<std::mutex> locker(lock_walls);

    int dmin = 10000;
    int xc = coordinates_.x;
    int yc = coordinates_.y;
    for (int i = 0; i < robots.size(); i++)
    {
        int xw = robots[i].x;
        int yw = robots[i].y;
        int dx = xw - xc;
        int dy = yw - yc;
        int d = dx * dx + dy * dy;
        if (d < dmin)
        {
            dmin = d;
        }
    }
    if (dmin == 0)
    {
        return 10000;
    }
    else if (dmin < 40)
    {
        return 1000 / dmin;
    }
    else
    {
        return 0;
    }
    /*
    int xc = coordinates_.x;
    int yc = coordinates_.y;
    
    for (int i = 0; i < robots.size(); i++)
    {
        int xw = robots[i].x;
        int yw = robots[i].y;
        if (abs(xw - xc) < 9 && abs(yw - yc) < 9)
        {
            return true;
        }
    }
    return false;*/
}


AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}