#include "states/StateStopped.h"
#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "states/DroneContext.h"
#include <cmath>
#include <memory>

auto StateStopped::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
    if (std::fabs(ctx.deltaAngle) > ctx.cfg.turnThreshold) {
        float rate = (ctx.deltaAngle > 0.0f ? 1.0f : -1.0f) * ctx.cfg.angularSpeed;
        ctx.command = {DroneState::Turning, rate};
        return std::make_unique<StateTurning>();
    }
    ctx.command = {DroneState::Accelerating, 0.0f};
    return std::make_unique<StateAccelerating>();
}

auto StateStopped::name() const -> const std::string& { return name_; }
auto StateStopped::stateId() const -> int { return id_; }
