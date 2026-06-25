#include "MissionProcessor.h"
#include "Logger.h"
#include "json/json.hpp"
#include <cmath>
#include <fstream>

using json = nlohmann::json;

constexpr int MAX_STEPS = 10000;

// ── Constructor / destructor ────────────────────────────────────────────────

MissionProcessor::MissionProcessor(ITargetProvider* t, IBallisticSolver* s, IConfigLoader* l)
    : targets_(t), solver_(s), loader_(l) {}

// ── Public API ──────────────────────────────────────────────────────────────

bool MissionProcessor::init() {
    cfg_  = loader_->getConfig();
    ammo_ = loader_->getAmmoParams();

    acceleration_     = calcAccel(cfg_.attackSpeed, cfg_.accelPath);
    currentPos_       = cfg_.initialPos;
    currentDir_       = cfg_.initialDir;
    currentSpeed_     = 0.0f;
    currentTime_      = 0.0f;
    currentTargetIdx_ = -1;
    droneState_       = STOPPED;
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
            && (droneState_ == MOVING || droneState_ == ACCELERATING);

        if (!solver_->solve(currentPos_, currentDir_, currentSpeed_,
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
    float desiredDir = std::atan2(bestFirePos.y - currentPos_.y,
                                   bestFirePos.x - currentPos_.x);
    float deltaAngle = normalizeAngle(desiredDir - currentDir_);
    float deltaPath  = 0.0f;

    // STEP 3: автомат станів дрона
    // Кожен крок симуляції:
    //   - оновлюємо швидкість і стан відповідно до поточного стану
    //   - рахуємо deltaPath — шлях пройдений за цей крок (метри)
    //   - (prevSpeed + currentSpeed) / 2 * dt — усереднена швидкість за крок
    //     дає точніший результат ніж просто speed * dt при прискоренні/гальмуванні
    switch (droneState_) {
        case STOPPED:
            currentSpeed_ = 0.0f;
            if (std::fabs(deltaAngle) > cfg_.turnThreshold)
                droneState_ = TURNING;
            else { currentDir_ = desiredDir; droneState_ = ACCELERATING; }
            break;

        case TURNING: {
            float da = normalizeAngle(desiredDir - currentDir_);
            if (std::fabs(da) <= cfg_.angularSpeed * cfg_.simTimeStep) {
                currentDir_ = desiredDir;
                droneState_ = ACCELERATING;
            } else {
                currentDir_ += (da > 0 ? 1.0f : -1.0f) * cfg_.angularSpeed * cfg_.simTimeStep;
                currentDir_  = normalizeAngle(currentDir_);
            }
            break;
        }

        case ACCELERATING: {
            if (std::fabs(deltaAngle) > cfg_.turnThreshold && currentSpeed_ > 0.0f) {
                droneState_   = DECELERATING;
                float prev    = currentSpeed_;
                currentSpeed_ -= acceleration_ * cfg_.simTimeStep;
                if (currentSpeed_ <= 0.0f) { currentSpeed_ = 0.0f; droneState_ = STOPPED; }
                deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
            } else {
                if (std::fabs(deltaAngle) <= cfg_.turnThreshold) currentDir_ = desiredDir;
                float prev    = currentSpeed_;
                currentSpeed_ += acceleration_ * cfg_.simTimeStep;
                if (currentSpeed_ >= cfg_.attackSpeed) {
                    currentSpeed_ = cfg_.attackSpeed;
                    droneState_   = MOVING;
                }
                deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
            }
            break;
        }

        case MOVING: {
            if (std::fabs(deltaAngle) > cfg_.turnThreshold) {
                droneState_   = DECELERATING;
                float prev    = currentSpeed_;
                currentSpeed_ -= acceleration_ * cfg_.simTimeStep;
                if (currentSpeed_ <= 0.0f) { currentSpeed_ = 0.0f; droneState_ = STOPPED; }
                deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
            } else {
                if (std::fabs(deltaAngle) <= cfg_.turnThreshold) currentDir_ = desiredDir;
                deltaPath = currentSpeed_ * cfg_.simTimeStep;
            }
            break;
        }

        case DECELERATING: {
            float prev    = currentSpeed_;
            currentSpeed_ -= acceleration_ * cfg_.simTimeStep;
            if (currentSpeed_ <= 0.0f) { currentSpeed_ = 0.0f; droneState_ = STOPPED; }
            deltaPath = (prev + currentSpeed_) / 2.0f * cfg_.simTimeStep;
            break;
        }

        default:
            LOG("Error: unknown drone state");
            done_ = true;
            return {};
    }

    // STEP 4: оновлення позиції
    currentPos_.x += std::cos(currentDir_) * deltaPath;
    currentPos_.y += std::sin(currentDir_) * deltaPath;

    // STEP 5: запис кроку і просування часу
    currentTime_ += cfg_.simTimeStep;
    SimStep s = {
        .pos             = currentPos_,
        .direction       = currentDir_,
        .state           = static_cast<int>(droneState_),
        .targetIdx       = currentTargetIdx_,
        .dropPoint       = bestFirePos,
        .aimPoint        = bestAimPoint,
        .predictedTarget = bestPredPos
    };
    steps_.push_back(s);

    DEBUG("Step: "    << steps_.size()
        << ", Time: " << currentTime_
        << ", Pos: (" << currentPos_.x << ", " << currentPos_.y << ")"
        << ", Dir: "  << currentDir_
        << ", State: "<< static_cast<int>(droneState_)
        << ", Target:"<< currentTargetIdx_);

    // STEP 6: перевірка умови скидання
    if (droneState_ == MOVING
            && calcDistance(currentPos_, bestFirePos) <= cfg_.hitRadius * 0.2f)
        done_ = true;

    return s;
}

void MissionProcessor::reset() {
    if (!initialized_) return;
    currentPos_       = cfg_.initialPos;
    currentDir_       = cfg_.initialDir;
    currentSpeed_     = 0.0f;
    currentTime_      = 0.0f;
    currentTargetIdx_ = -1;
    droneState_       = STOPPED;
    done_             = false;
    steps_.clear();
}

void MissionProcessor::changeSolver(IBallisticSolver* s) { solver_ = s; }

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
