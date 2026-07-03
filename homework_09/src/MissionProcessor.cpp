#include "MissionProcessor.h"
#include "states/StateStopped.h"
#include "Logger.h"
#include "json/json.hpp"
#include <cmath>
#include <fstream>
#include <memory>

using json = nlohmann::json;

constexpr int MAX_STEPS = 10000;

// ── Constructor / destructor ────────────────────────────────────────────────

MissionProcessor::MissionProcessor(std::unique_ptr<ITargetProvider> t, std::unique_ptr<IBallisticSolver> s, std::unique_ptr<IConfigLoader> l)
    : targets_(std::move(t)), solver_(std::move(s)), loader_(std::move(l)) {}

// ── Public API ──────────────────────────────────────────────────────────────

bool MissionProcessor::init() {
    cfg_  = loader_->getConfig();
    ammo_ = loader_->getAmmoParams();

    ctx_.acceleration = calcAccel(cfg_.attackSpeed, cfg_.accelPath);
    ctx_.pos          = cfg_.initialPos;
    ctx_.direction    = cfg_.initialDir;
    ctx_.speed        = 0.0f;
    currentTime_      = 0.0f;
    currentTargetIdx_ = -1;
    state_            = std::make_unique<StateStopped>();
    done_             = false;

    steps_.clear();
    steps_.reserve(MAX_STEPS);

    initialized_ = true;
    return true;
}

bool MissionProcessor::hasNext() const {
    return initialized_ && !done_ && static_cast<int>(steps_.size()) < MAX_STEPS;
}

SimStep MissionProcessor::step() {
    // STEP 1: вибір найближчої цілі
    // Для кожної цілі solver ітеративно уточнює прогноз позиції та час польоту.
    // Обираємо ціль з найменшим сумарним часом польоту дрона.
    int   bestTarget   = -1;
    float bestTime     = 1e30f;
    Coord bestFirePos  = {};
    Coord bestPredPos  = {};
    Coord bestAimPoint = {};

    for (int i = 0; i < targets_->getTargetCount(); ++i) {
        float totalTime = 0.0f;
        Coord firePos{}, interPos{}, predPos{}, aimPoint{};
        bool  hasInter = false;

        bool alreadyApproaching = (i == currentTargetIdx_)
            && (state_->name() == "Moving" || state_->name() == "Accelerating");

        if (!solver_->solve(ctx_.pos, ctx_.direction, ctx_.speed,
                targets_->getTarget(i), currentTime_, alreadyApproaching,
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

    if (bestTarget == -1) {
        LOG("Error: no valid target found at time " << currentTime_);
        done_ = true;
        return {};
    }
    currentTargetIdx_ = bestTarget;

    // STEP 2: бажаний напрямок і кут повороту
    float desiredDir = std::atan2(bestFirePos.y - ctx_.pos.y,
                                   bestFirePos.x - ctx_.pos.x);
    ctx_.desiredDir = desiredDir;
    ctx_.deltaAngle = normalizeAngle(desiredDir - ctx_.direction);

    // STEP 3: автомат станів дрона (патерн State)
    // Поточний стан сам оновлює швидкість/напрямок у ctx_ і обчислює deltaPath
    //   - deltaPath — шлях пройдений за цей крок (метри)
    //   - (prevSpeed + currentSpeed) / 2 * dt — усереднена швидкість за крок
    //     дає точніший результат ніж просто speed * dt при прискоренні/гальмуванні
    // execute() повертає наступний стан або nullptr (стан не змінився).
    ctx_.deltaPath = 0.0f;
    if (auto next = state_->execute(ctx_))
        state_ = std::move(next);

    // STEP 4: оновлення позиції
    ctx_.pos.x += std::cos(ctx_.direction) * ctx_.deltaPath;
    ctx_.pos.y += std::sin(ctx_.direction) * ctx_.deltaPath;

    // STEP 5: запис кроку і просування часу
    currentTime_ += cfg_.simTimeStep;
    SimStep s = {
        .pos             = ctx_.pos,
        .direction       = ctx_.direction,
        .state           = state_->stateId(),
        .targetIdx       = currentTargetIdx_,
        .dropPoint       = bestFirePos,
        .aimPoint        = bestAimPoint,
        .predictedTarget = bestPredPos
    };
    steps_.push_back(s);

    DEBUG("Step: "    << steps_.size()
        << ", Time: " << currentTime_
        << ", Pos: (" << ctx_.pos.x << ", " << ctx_.pos.y << ")"
        << ", Dir: "  << ctx_.direction
        << ", State: "<< state_->stateId()
        << ", Target:"<< currentTargetIdx_);

    // STEP 6: перевірка умови скидання
    if (state_->name() == "Moving"
            && calcDistance(ctx_.pos, bestFirePos) <= cfg_.hitRadius * 0.2f)
        done_ = true;

    return s;
}

void MissionProcessor::reset() {
    if (!initialized_) return;
    ctx_.pos          = cfg_.initialPos;
    ctx_.direction    = cfg_.initialDir;
    ctx_.speed        = 0.0f;
    currentTime_      = 0.0f;
    currentTargetIdx_ = -1;
    state_            = std::make_unique<StateStopped>();
    done_             = false;
    steps_.clear();
}

void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> s) { solver_ = std::move(s); }

int  MissionProcessor::getRecordedSteps() const { return static_cast<int>(steps_.size()); }
bool MissionProcessor::isDone()           const { return done_; }

// ── Output ──────────────────────────────────────────────────────────────────

bool MissionProcessor::writeOutput(const std::string& filename) const {
    std::fstream f(filename, std::ios::out);
    if (!f) { LOG("Error opening output file: " << filename); return false; }

    json out;
    out["totalSteps"] = steps_.size();
    out["steps"]      = json::array();

    for (const auto& s : steps_) {
        json step;
        step["position"]        = { {"x", s.pos.x},             {"y", s.pos.y} };
        step["direction"]       = s.direction;
        step["state"]           = s.state;
        step["targetIndex"]     = s.targetIdx;
        step["dropPoint"]       = { {"x", s.dropPoint.x},       {"y", s.dropPoint.y} };
        step["aimPoint"]        = { {"x", s.aimPoint.x},        {"y", s.aimPoint.y} };
        step["predictedTarget"] = { {"x", s.predictedTarget.x}, {"y", s.predictedTarget.y} };
        out["steps"].push_back(step);
    }

    f << out.dump(2);
    f.close();
    return true;
}
