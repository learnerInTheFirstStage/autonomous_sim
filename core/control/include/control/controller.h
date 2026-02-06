#pragma once

#include "common/point.h"
#include "planner/path.h"
#include "control/control_command.h"
#include "vehicle/vehicle_model.h"

namespace control {

class Controller {

public:
    virtual ~Controller() = default;

    virtual ControlCommand ComputeCommand(
        const vehicle::VehicleState& state,
        const planner::Path& path) = 0;
};

}  // namespace control