#include "MissionPlanner.h"
#include "states/StateStopped.h"
#include "states/DroneContext.h"
#include "MathUtils.h"

#include <cmath>
#include <utility>

namespace {
constexpr float DROP_RATIO = 0.2f;
}

MissionPlanner::MissionPlanner(std::unique_ptr<IBallisticSolver> solver,
                               DroneConfig cfg, AmmoParams ammo)
    : solver_(std::move(solver)), cfg_(std::move(cfg)), ammo_(std::move(ammo)),
      acceleration_(calcAccel(cfg_.attackSpeed, cfg_.accelPath)),
      state_(std::make_unique<StateStopped>()) {}

auto MissionPlanner::plan(const DroneTelemetry& tel, const ITargetProvider& targets) -> PlanResult {
    const float speed = length(tel.speed);

    // STEP 1: ціль з найменшим часом польоту дрона.
    int   bestTarget   = -1;
    float bestTime     = 1e30f;
    Coord bestFirePos  = {};
    Coord bestPredPos  = {};
    Coord bestAimPoint = {};

    for (int i = 0; i < targets.getTargetCount(); ++i) {
        float totalTime = 0.0f;
        Coord firePos{}, interPos{}, predPos{}, aimPoint{};
        bool  hasInter = false;
        bool  alreadyApproaching = (i == currentTargetIdx_)
            && (state_->name() == "Moving" || state_->name() == "Accelerating");

        if (!solver_->solve(tel.pos, tel.direction, speed,
                targets.getTarget(i), 0.0f, alreadyApproaching,
                cfg_, ammo_, firePos, interPos, hasInter, totalTime,
                predPos, aimPoint))
            continue;

        if (totalTime < bestTime) {
            bestTime     = totalTime;
            bestTarget   = i;
            bestFirePos  = firePos;
            bestPredPos  = predPos;
            bestAimPoint = aimPoint;
        }
    }

    if (bestTarget == -1) return PlanResult{};   // немає цілі → тримати нейтраль
    currentTargetIdx_ = bestTarget;

    // STEP 2: бажаний напрямок.
    const float desiredDir = std::atan2(bestFirePos.y - tel.pos.y, bestFirePos.x - tel.pos.x);

    // STEP 3: стейт-машина формує команду.
    DroneContext ctx{cfg_};
    ctx.acceleration = acceleration_;
    ctx.pos          = tel.pos;
    ctx.direction    = tel.direction;
    ctx.speed        = speed;
    ctx.desiredDir   = desiredDir;
    ctx.deltaAngle   = normalizeAngle(desiredDir - tel.direction);
    if (auto next = state_->execute(ctx)) state_ = std::move(next);

    // STEP 4: результат такту.
    PlanResult r;
    r.valid   = true;
    r.command = ctx.command;
    r.log     = SimStep{
        .pos               = tel.pos,
        .direction         = tel.direction,
        .state             = state_->stateId(),
        .targetIdx         = currentTargetIdx_,
        .dropPoint         = bestFirePos,
        .aimPoint          = bestAimPoint,
        .predictedTarget   = bestPredPos,
        .timeSecSinceStart = tel.timeSecSinceStart,
    };

    // STEP 5: умова скидання.
    r.drop = (state_->name() == "Moving"
              && calcDistance(tel.pos, bestFirePos) <= cfg_.hitRadius * DROP_RATIO);
    return r;
}
