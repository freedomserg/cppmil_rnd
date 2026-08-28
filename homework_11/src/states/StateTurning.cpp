#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "states/DroneContext.h"
#include <cmath>
#include <memory>

auto StateTurning::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
    if (std::fabs(ctx.deltaAngle) <= ctx.cfg.angularSpeed * ctx.cfg.simTimeStep) {
        ctx.command = {DroneState::Accelerating, 0.0f};
        return std::make_unique<StateAccelerating>();
    }
    float rate = (ctx.deltaAngle > 0.0f ? 1.0f : -1.0f) * ctx.cfg.angularSpeed;
    ctx.command = {DroneState::Turning, rate};
    return nullptr;
}

auto StateTurning::name() const -> const std::string& { return name_; }
auto StateTurning::stateId() const -> int { return id_; }
