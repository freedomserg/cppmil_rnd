#include "states/StateTurning.h"
#include <memory>
#include "states/StateAccelerating.h"
#include "states/DroneContext.h"

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx) {
    float da = normalizeAngle(ctx.desiredDir - ctx.direction);
    if (std::fabs(da) <= ctx.cfg.angularSpeed * ctx.cfg.simTimeStep) {
        ctx.direction = ctx.desiredDir;
        return std::make_unique<StateAccelerating>();
    } else {
        ctx.direction += (da > 0 ? 1.0f : -1.0f) * ctx.cfg.angularSpeed * ctx.cfg.simTimeStep;
        ctx.direction  = normalizeAngle(ctx.direction);
        return nullptr;   // лишаємось у TURNING
    }
}

const std::string& StateTurning::name() const { return name_; }
int StateTurning::stateId() const { return id_; }
