#include "states/StateMoving.h"
#include "states/StateDecelerating.h"
#include "states/DroneContext.h"
#include <cmath>
#include <memory>

auto StateMoving::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
    if (std::fabs(ctx.deltaAngle) > ctx.cfg.turnThreshold) {
        ctx.command = {DroneState::Decelerating, 0.0f};
        return std::make_unique<StateDecelerating>();
    }
    ctx.command = {DroneState::Moving, 0.0f};
    return nullptr;
}

auto StateMoving::name() const -> const std::string& { return name_; }
auto StateMoving::stateId() const -> int { return id_; }
