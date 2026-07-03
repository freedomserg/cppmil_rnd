#include "states/StateDecelerating.h"
#include "states/StateStopped.h"
#include "states/DroneContext.h"
#include <memory>

auto StateDecelerating::execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> {
    constexpr float kSpeedEps = 1e-3f;
    if (ctx.speed <= kSpeedEps) {
        ctx.command = {DroneState::Stopped, 0.0f};
        return std::make_unique<StateStopped>();
    }
    ctx.command = {DroneState::Decelerating, 0.0f};
    return nullptr;
}

auto StateDecelerating::name() const -> const std::string& { return name_; }
auto StateDecelerating::stateId() const -> int { return id_; }
