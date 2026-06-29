#include "MissionProcessor.h"
#include "physics/DronePhysics.h"
#include "interfaces/ITargetProvider.h"
#include "states/StateStopped.h"
#include "states/DroneContext.h"
#include "MathUtils.h"
#include "Logger.h"
#include <json.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <thread>

using json = nlohmann::json;

constexpr int   MAX_STEPS  = 10000;
constexpr float DROP_RATIO = 0.2f;

MissionProcessor::MissionProcessor(ITargetProvider* provider, DronePhysics* physics,
                                   std::unique_ptr<IBallisticSolver> solver,
                                   const DroneConfig& cfg, const AmmoParams& ammo)
    : provider_(provider), physics_(physics), solver_(std::move(solver)),
      cfg_(cfg), ammo_(ammo),
      acceleration_(calcAccel(cfg.attackSpeed, cfg.accelPath)),
      state_(std::make_unique<StateStopped>()) {
    steps_.reserve(MAX_STEPS);
}

void MissionProcessor::start() { started_ = true; }
void MissionProcessor::stop()  { running_ = false; }
auto MissionProcessor::isThreadReady() const -> bool { return ready_; }
auto MissionProcessor::isDone() const -> bool { return done_; }
auto MissionProcessor::getRecordedSteps() const -> int { return static_cast<int>(steps_.size()); }

void MissionProcessor::run() {
    ready_ = true;
    while (!started_ && running_)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    while (running_ && !done_) {
        if (!step()) break;
        std::this_thread::sleep_for(
            std::chrono::duration<float>(cfg_.simTimeStep / cfg_.timeScale));
    }
    done_ = true;
}

auto MissionProcessor::step() -> bool {
    const DroneTelemetry tel = physics_->getTelemetry();
    const float speed = length(tel.speed);

    // STEP 1: ціль з найменшим часом польоту дрона.
    int   bestTarget   = -1;
    float bestTime     = 1e30f;
    Coord bestFirePos  = {};
    Coord bestPredPos  = {};
    Coord bestAimPoint = {};

    for (int i = 0; i < provider_->getTargetCount(); ++i) {
        float totalTime = 0.0f;
        Coord firePos{}, interPos{}, predPos{}, aimPoint{};
        bool  hasInter = false;
        bool  alreadyApproaching = (i == currentTargetIdx_)
            && (state_->name() == "Moving" || state_->name() == "Accelerating");

        if (!solver_->solve(tel.pos, tel.direction, speed,
                provider_->getTarget(i), 0.0f, alreadyApproaching,
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

    if (bestTarget == -1) { LOG("Error: no valid target found"); return false; }
    currentTargetIdx_ = bestTarget;

    // STEP 2: бажаний напрямок.
    const float desiredDir = std::atan2(bestFirePos.y - tel.pos.y, bestFirePos.x - tel.pos.x);

    // STEP 3: стейт-машина формує команду фізиці.
    DroneContext ctx{cfg_};
    ctx.acceleration = acceleration_;
    ctx.pos          = tel.pos;
    ctx.direction    = tel.direction;
    ctx.speed        = speed;
    ctx.desiredDir   = desiredDir;
    ctx.deltaAngle   = normalizeAngle(desiredDir - tel.direction);
    if (auto next = state_->execute(ctx)) state_ = std::move(next);

    // STEP 4: команда фізиці.
    physics_->pushCommand(ctx.command);

    // STEP 5: лог кроку (стан і час — з телеметрії фізики).
    steps_.push_back(SimStep{
        .pos               = tel.pos,
        .direction         = tel.direction,
        .state             = state_->stateId(),
        .targetIdx         = currentTargetIdx_,
        .dropPoint         = bestFirePos,
        .aimPoint          = bestAimPoint,
        .predictedTarget   = bestPredPos,
        .timeSecSinceStart = tel.timeSecSinceStart,
    });

    // STEP 6: умова скидання.
    if (state_->name() == "Moving"
            && calcDistance(tel.pos, bestFirePos) <= cfg_.hitRadius * DROP_RATIO)
        return false;

    return static_cast<int>(steps_.size()) < MAX_STEPS;
}

auto MissionProcessor::writeOutput(const std::string& filename) const -> bool {
    std::ofstream f(filename);
    if (!f) { LOG("Error opening output file: " << filename); return false; }

    json out;
    out["totalSteps"] = steps_.size();
    out["steps"]      = json::array();

    for (const auto& s : steps_) {
        json step;
        step["position"]          = { {"x", s.pos.x},             {"y", s.pos.y} };
        step["direction"]         = s.direction;
        step["state"]             = s.state;
        step["targetIndex"]       = s.targetIdx;
        step["dropPoint"]         = { {"x", s.dropPoint.x},       {"y", s.dropPoint.y} };
        step["aimPoint"]          = { {"x", s.aimPoint.x},        {"y", s.aimPoint.y} };
        step["predictedTarget"]   = { {"x", s.predictedTarget.x}, {"y", s.predictedTarget.y} };
        step["timeSecSinceStart"] = s.timeSecSinceStart;
        out["steps"].push_back(step);
    }

    f << out.dump(2);
    return true;
}
