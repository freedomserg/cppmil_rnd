#include "states/StateMoving.h"
#include "states/StateDecelerating.h"
#include "states/StateStopped.h"
#include "states/DroneContext.h"

std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx) {
    // Потрібен розворот — починаємо гальмувати.
    if (std::fabs(ctx.deltaAngle) > ctx.cfg.turnThreshold) {
        float prev = ctx.speed;
        ctx.speed -= ctx.acceleration * ctx.cfg.simTimeStep;
        if (ctx.speed <= 0.0f) {
            ctx.speed     = 0.0f;
            ctx.deltaPath = (prev + ctx.speed) / 2.0f * ctx.cfg.simTimeStep;
            return std::make_unique<StateStopped>();
        }
        ctx.deltaPath = (prev + ctx.speed) / 2.0f * ctx.cfg.simTimeStep;
        return std::make_unique<StateDecelerating>();
    }

    // Рух на макс. швидкості.
    if (std::fabs(ctx.deltaAngle) <= ctx.cfg.turnThreshold) ctx.direction = ctx.desiredDir;
    ctx.deltaPath = ctx.speed * ctx.cfg.simTimeStep;
    return nullptr;   // лишаємось у MOVING
}

const std::string& StateMoving::name() const { return name_; }
int StateMoving::stateId() const { return id_; }
