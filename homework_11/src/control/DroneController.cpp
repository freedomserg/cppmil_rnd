#include "control/DroneController.h"

#include <algorithm>

namespace {
constexpr float kSpeedEps = 1e-3f;
}

auto DroneController::toControl(const DroneCommand& cmd, float speed) const -> Output {
    // turnRate: +angleSpeed = +deltaAngle = поворот вліво = +turnRate (як у чекера).
    float turnRate = 0.0f;
    if (cfg_.angularSpeed > kSpeedEps)
        turnRate = std::clamp(cmd.angleSpeed / cfg_.angularSpeed, -1.0f, 1.0f);

    float accel = 0.0f;
    switch (cmd.state) {
    case DroneState::Accelerating: accel =  1.0f; break;
    case DroneState::Decelerating: accel = -1.0f; break;
    case DroneState::Stopped:      accel = (speed > kSpeedEps) ? -1.0f : 0.0f; break;
    case DroneState::Moving:
    case DroneState::Turning:      accel =  0.0f; break;
    }

    return Output{accel, turnRate};
}
