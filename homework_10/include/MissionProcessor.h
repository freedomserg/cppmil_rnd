#pragma once

#include "interfaces/IBallisticSolver.h"
#include "states/IDroneState.h"
#include "Types.h"
#include <atomic>
#include <memory>
#include <string>
#include <vector>

class ITargetProvider;
class DronePhysics;

class MissionProcessor {
public:
    MissionProcessor(ITargetProvider* provider, DronePhysics* physics,
                     std::unique_ptr<IBallisticSolver> solver,
                     const DroneConfig& cfg, const AmmoParams& ammo);

    void run();                         // тіло потоку
    void start();
    void stop();
    [[nodiscard]] auto isThreadReady() const -> bool;
    [[nodiscard]] auto isDone() const -> bool;

    [[nodiscard]] auto writeOutput(const std::string& filename) const -> bool;
    [[nodiscard]] auto getRecordedSteps() const -> int;

private:
    auto step() -> bool;                // один крок планування; false — місія завершена

    ITargetProvider* provider_;         // non-owning
    DronePhysics*    physics_;          // non-owning
    std::unique_ptr<IBallisticSolver> solver_;
    DroneConfig cfg_;
    AmmoParams  ammo_;
    float acceleration_ = 0.0f;
    std::unique_ptr<IDroneState> state_;

    int currentTargetIdx_ = -1;
    std::vector<SimStep> steps_;

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> running_{true};
    std::atomic<bool> done_{false};
};
