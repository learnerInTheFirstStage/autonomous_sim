#pragma once

namespace planner {

struct AStarNode
{
    int x;
    int y;

    double g = 0.0;  // cost from start
    double h = 0.0;  // heuristic to goal
    double f = 0.0;  // g + h

    AStarNode* parent = nullptr;
    
    AStarNode(int x, int y) : x(x), y(y) {}
};


}  // namespace planner