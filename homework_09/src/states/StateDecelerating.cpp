#include "states/StateDecelerating.h"
#include "states/StateStopped.h"
#include "states/DroneContext.h"

std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx) {
    float prev = ctx.speed;
    ctx.speed -= ctx.acceleration * ctx.cfg.simTimeStep;
    if (ctx.speed <= 0.0f) {
        ctx.speed     = 0.0f;
        ctx.deltaPath = (prev + ctx.speed) / 2.0f * ctx.cfg.simTimeStep;
        return std::make_unique<StateStopped>();
    }
    ctx.deltaPath = (prev + ctx.speed) / 2.0f * ctx.cfg.simTimeStep;
    return nullptr;   // лишаємось у DECELERATING
}

const std::string& StateDecelerating::name() const { return name_; }
int StateDecelerating::stateId() const { return id_; }
