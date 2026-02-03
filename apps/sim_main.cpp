#include <iostream>

#include "planner/astar/astar_planner.h"
#include "map/grid_map.h"
#include "control/pure_pursuit.h"

int main() {
    map::GridMap map(10, 10, 1.0);

    map.SetOccupied(3, 3);
    map.SetOccupied(3, 4);
    map.SetOccupied(3, 5);

    planner::AStarPlanner planner;
    planner::Path path;

    common::Point2D start({0, 0});
    common::Point2D goal({7, 7});

    if (!planner.Plan(map, start, goal, path)) {
        std::cout << "Planning failed" << std::endl;
        return 0;
    } 

    control::PurePursuitController controller(1.5);

    common::Point2D current = start;

    for (int i = 0; i < 10; i ++ ) {
        auto cmd = controller.ComputeCommand(current, path);

        std::cout << "Step " << i
                  << " steering: " << cmd.steering_angle
                  << " speed: " << cmd.speed << std::endl;

        // Simple simulation: Move one step ahead to goal
        current.x += std::cos(cmd.steering_angle);
        current.y += std::sin(cmd.steering_angle);
    }

    return 0;
}