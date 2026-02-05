#include <iostream>

#include "planner/astar/astar_planner.h"
#include "map/grid_map.h"
#include "control/pure_pursuit.h"
#include "vehicle/vehicle_model.h"

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

    vehicle::VehicleState state;
    state.x = start.x;
    state.y = start.y;
    state.heading = 0.0;
    state.velocity = 0.0;

    vehicle::VehicleModel vehicle_model;

    for (int i = 0; i < 20; i ++ ) {
        auto cmd = controller.ComputeCommand(
            common::Point2D({state.x, state.y}), path);
        
        vehicle::ControlCommand vcmd;
        vcmd.steering_angle = cmd.steering_angle;
        vcmd.speed = cmd.speed;

        vehicle_model.Update(state, vcmd);

        std::cout << "Step " << i
                  << " x: " << state.x
                  << " y: " << state.y
                  << " heading: " << state.heading
                  << std::endl;
    }

    return 0;
}