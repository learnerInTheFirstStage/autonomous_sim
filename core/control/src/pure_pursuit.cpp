#include "control/pure_pursuit.h"
#include <cmath>

namespace control {

PurePursuitController::PurePursuitController(double lookahead_distance)
    : lookahead_distance_(lookahead_distance) {}


ControlCommand PurePursuitController::ComputeCommand(
    const common::Point2D& current_pose,
    const planner::Path& path) {

    ControlCommand cmd;

    if (path.points.empty()) return cmd;

    // Find lookahead point
    common::Point2D target = path.points.back();

    for (const auto& p : path.points) {
        double dx = p.x - current_pose.x;
        double dy = p.y - current_pose.y;

        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist >= lookahead_distance_) {
            target = p;
            break;
        }
    }

    double dx = target.x - current_pose.x;
    double dy = target.y - current_pose.y;

    cmd.steering_angle = std::atan2(dy, dx);
    cmd.speed = 1.0;  // Hardcoded First

    return cmd;

}

}  // namespace control