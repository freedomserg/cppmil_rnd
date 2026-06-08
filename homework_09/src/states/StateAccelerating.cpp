#include "states/StateAccelerating.h"
#include "states/StateDecelerating.h"
#include "states/StateMoving.h"
#include "states/StateStopped.h"
#include "states/DroneContext.h"

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx) {
    // Потрібен розворот і дрон ще рухається — гальмуємо.
    if (std::fabs(ctx.deltaAngle) > ctx.cfg.turnThreshold && ctx.speed > 0.0f) {
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

    // Розгін у напрямку цілі.
    if (std::fabs(ctx.deltaAngle) <= ctx.cfg.turnThreshold) ctx.direction = ctx.desiredDir;
    float prev = ctx.speed;
    ctx.speed += ctx.acceleration * ctx.cfg.simTimeStep;
    if (ctx.speed >= ctx.cfg.attackSpeed) {
        ctx.speed     = ctx.cfg.attackSpeed;
        ctx.deltaPath = (prev + ctx.speed) / 2.0f * ctx.cfg.simTimeStep;
        return std::make_unique<StateMoving>();
    }
    ctx.deltaPath = (prev + ctx.speed) / 2.0f * ctx.cfg.simTimeStep;
    return nullptr;   // лишаємось у ACCELERATING
}

const std::string& StateAccelerating::name() const { return name_; }
int StateAccelerating::stateId() const { return id_; }
