#include "vehicle/vehicle_model.h"
#include <cmath>

namespace vehicle {

VehicleModel::VehicleModel(double wheelbase, double dt)
    : wheelbase_(wheelbase), dt_(dt) {}

void VehicleModel::Update(VehicleState& state, const ControlCommand& cmd) {
    state.velocity = cmd.speed;

    state.x += state.velocity * std::cos(state.heading) * dt_;
    state.y += state.velocity * std::sin(state.heading) * dt_;
    state.heading += (state.velocity / wheelbase_) * std::tan(cmd.steering_angle) * dt_;
}

}  // namespace vehicle