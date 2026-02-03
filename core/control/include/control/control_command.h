#pragma once

namespace control {

struct ControlCommand {
    double steering_angle{0.0};
    double speed{0.0};
};

}  // namespace control