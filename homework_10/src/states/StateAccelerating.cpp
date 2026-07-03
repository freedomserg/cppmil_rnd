#include "states/StateAccelerating.h"
#include "states/StateDecelerating.h"
#include "states/StateMoving.h"
#include "states/StateStopped.h"
#include "states/DroneContext.h"
#include <cmath>
#include <memory>

auto StateAccelerating::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
    constexpr float kSpeedEps = 1e-3f;
    if (std::fabs(ctx.deltaAngle) > ctx.cfg.turnThreshold) {
        if (ctx.speed > kSpeedEps) {
            ctx.command = {DroneState::Decelerating, 0.0f};
            return std::make_unique<StateDecelerating>();
        }
        ctx.command = {DroneState::Stopped, 0.0f};
        return std::make_unique<StateStopped>();
    }
    if (ctx.speed >= ctx.cfg.attackSpeed - kSpeedEps) {
        ctx.command = {DroneState::Moving, 0.0f};
        return std::make_unique<StateMoving>();
    }
    ctx.command = {DroneState::Accelerating, 0.0f};
    return nullptr;
}

auto StateAccelerating::name() const -> const std::string& { return name_; }
auto StateAccelerating::stateId() const -> int { return id_; }
