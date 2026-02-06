#include "control/pure_pursuit.h"
#include <cmath>
#include "vehicle/vehicle_model.h"

namespace control {

PurePursuitController::PurePursuitController(double lookahead_distance)
    : lookahead_distance_(lookahead_distance) {}


ControlCommand PurePursuitController::ComputeCommand(
    const vehicle::VehicleState& state,
    const planner::Path& path) {

    ControlCommand cmd;

    if (path.points.empty()) return cmd;

    // Find lookahead point
    common::Point2D target = path.points.back();

    for (const auto& p : path.points) {
        double dx = p.x - state.x;
        double dy = p.y - state.y;

        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= lookahead_distance_) {
            target = p;
            break;
        }
    }

    double dx = target.x - state.x;
    double dy = target.y - state.y;

    double target_angle = std::atan2(dy, dx);

    double alpha = target_angle - state.heading;

    double L = 2.5;

    cmd.steering_angle = std::atan2(2.0 * L * std::sin(alpha),
                                    lookahead_distance_);

    double max_steer = 0.4;
    if (cmd.steering_angle > max_steer)
        cmd.steering_angle = max_steer;
    if (cmd.steering_angle < -max_steer)
        cmd.steering_angle = -max_steer;

    // More advanced way to control speed
    cmd.speed = std::max(0.3, 1.0 - std::abs(cmd.steering_angle));

    return cmd;

}

}  // namespace control