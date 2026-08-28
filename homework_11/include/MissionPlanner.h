#pragma once

#include "Types.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "states/IDroneState.h"

#include <memory>

// Результат одного планувального такту.
struct PlanResult {
    DroneCommand command{};   // {state, angleSpeed} для DroneController
    bool valid{false};        // false → немає розв'язної цілі, тримати нейтраль
    bool drop{false};         // true → скид зараз (перший true зараховується)
    SimStep log{};            // трасування кроку
};

// Headless-планувальник наведення: порт MissionProcessor::step() без потоків і
// фізики. Мутує лише внутрішній стан FSM; телеметрія і цілі — вхідні параметри.
class MissionPlanner {
public:
    MissionPlanner(std::unique_ptr<IBallisticSolver> solver, DroneConfig cfg, AmmoParams ammo);

    auto plan(const DroneTelemetry& tel, const ITargetProvider& targets) -> PlanResult;

private:
    std::unique_ptr<IBallisticSolver> solver_;
    DroneConfig cfg_;
    AmmoParams  ammo_;
    float acceleration_ = 0.0f;
    std::unique_ptr<IDroneState> state_;
    int   currentTargetIdx_ = -1;
};
