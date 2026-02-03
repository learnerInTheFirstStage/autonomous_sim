#pragma once

#include "common/point.h"
#include "planner/path.h"
#include "control/control_command.h"

namespace control {

class Controller {

public:
    virtual ~Controller() = default;

    virtual ControlCommand ComputeCommand(
        const common::Point2D& current_pose,
        const planner::Path& path) = 0;
};

}  // namespace control