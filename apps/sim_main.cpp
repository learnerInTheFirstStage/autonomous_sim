#include <iostream>
#include "planner/astar/astar_planner.h"
#include "map/grid_map.h"

int main() {
    map::GridMap map(10, 10, 1.0);
    map.SetOccupied(3, 3);
    map.SetOccupied(3, 4);
    map.SetOccupied(3, 5);

    planner::AStarPlanner planner;
    planner::Path path;

    common::Point2D start({0, 0});
    common::Point2D goal({7, 7});

    if (planner.Plan(map, start, goal, path)) {
        for (auto& p : path.points) {
            std::cout << "(" << p.x << ", " << p.y << ")\n";
        }
    } else {
        std::cout << "Planning failed\n";
    }
}