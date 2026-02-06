#pragma once

#include "control/controller.h"
#include "vehicle/vehicle_model.h"

namespace control {

class PurePursuitController : public Controller {

public:
    explicit PurePursuitController(double lookahead_distance);

    ControlCommand ComputeCommand(
        const vehicle::VehicleState& state,
        const planner::Path& path);

private:
    double lookahead_distance_;

};


}  // namespace control