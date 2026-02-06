#include <iostream>
#include <fstream>
#include <vector>

#include "planner/astar/astar_planner.h"
#include "map/grid_map.h"
#include "control/pure_pursuit.h"
#include "vehicle/vehicle_model.h"

int main() {
    map::GridMap map(10, 10, 1.0);
    std::vector<common::Point2D> trajectory;

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

    control::PurePursuitController controller(1.0);

    vehicle::VehicleState state;
    state.x = start.x;
    state.y = start.y;
    state.heading = 0.0;
    state.velocity = 0.0;

    vehicle::VehicleModel vehicle_model;

    for (int i = 0; i < 180; i ++ ) {
        auto cmd = controller.ComputeCommand(state, path);
        
        vehicle::ControlCommand vcmd;
        vcmd.steering_angle = cmd.steering_angle;
        vcmd.speed = cmd.speed;

        vehicle_model.Update(state, vcmd);

        trajectory.push_back({state.x, state.y});

        std::cout << "Step " << i
                  << " x: " << state.x
                  << " y: " << state.y
                  << " heading: " << state.heading
                  << std::endl;
    }

    // Create output directory manually or ensure it exists
    std::ofstream path_file("path.txt");
    for (auto& p : path.points) {
        path_file << p.x << " " << p.y << std::endl;
    }
    path_file.close();

    std::ofstream traj_file("trajectory.txt");
    for (auto& p : trajectory) {
        traj_file << p.x << " " << p.y << std::endl;
    }
    traj_file.close();

    std::cout << "Data exported for visualization." << std::endl;

    return 0;
}