#include "states/StateStopped.h"
#include "states/StateTurning.h"
#include "states/StateAccelerating.h"
#include "states/DroneContext.h"

std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx) {
    ctx.speed = 0.0f;
    if (std::fabs(ctx.deltaAngle) > ctx.cfg.turnThreshold)
        return std::make_unique<StateTurning>();
    ctx.direction = ctx.desiredDir;
    return std::make_unique<StateAccelerating>();
}

const std::string& StateStopped::name() const { return name_; }
int StateStopped::stateId() const { return id_; }