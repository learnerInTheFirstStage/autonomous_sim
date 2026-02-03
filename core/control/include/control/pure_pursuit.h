#pragma once

#include "control/controller.h"

namespace control {

class PurePursuitController : public Controller {

public:
    explicit PurePursuitController(double lookahead_distance);

    ControlCommand ComputeCommand(
        const common::Point2D& current_pose,
        const planner::Path& path) override;

private:
    double lookahead_distance_;

};


}  // namespace control