#pragma once

namespace vehicle {

struct VehicleState {
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double velocity{0.0};
};

struct ControlCommand {
    double steering_angle{0.0};
    double speed{0.0};
};

class VehicleModel {
public:
    VehicleModel(double wheelbase = 2.5, double dt = 0.1);

    void Update(VehicleState& state, const ControlCommand& cmd);

private:
    double wheelbase_;
    double dt_;
};

}  // namespace vehicle