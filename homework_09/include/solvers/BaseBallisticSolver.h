#pragma once

#include "interfaces/IBallisticSolver.h"

struct DroneConfig;
struct AmmoParams;
struct Coord;
struct Target;

// Спільна логіка всіх балістичних солверів: геометрія місії
// (прогноз цілі, маневрування дрона, вибір точки скиду) та solve().
// Нащадки відрізняються лише тим, звідки беруть два числа —
// ammoFlightTime_ (час польоту боєприпасу) та hDist_ (горизонтальна
// дистанція) — і заповнюють їх у власному конструкторі.
class BaseBallisticSolver : public IBallisticSolver {
    protected:
        float ammoFlightTime_ = 0.0f;
        float hDist_          = 0.0f;
        bool  ready_          = false;

        // Визначає точку скиду та якщо потрібно проміжну точку розгону.
        void getIntermediateAndDropPoint(
            const Coord& dronePos, const Coord& targetPos,
            float hDist, float accelPath,
            Coord& outIntermediatePos, Coord& outFirePos, bool& outHasIntermediate);

        // Знаходить точну позицію цілі в момент часу t методом лінійної інтерполяції.
        // Масив є циклічним — після останньої точки знову йде перша.
        Coord interpolate(const Target& target, float t, float arrayTimeStep);

        // Прогнозує де буде ціль через dt секунд від currentTime.
        // прогноз = поточна_позиція + швидкість * dt
        Coord extrapolate(const Target& target, float currentTime, float dt, float arrayTimeStep);

        bool needStop(float desiredDir, float currentDir, float angleThreshold);

        float calcTimeOfFlight(
            Coord currentPos, const Coord& targetPos,
            float currentDir, float angleThreshold, float angularSpeed,
            float currentSpeed, float maxSpeed, float accelPath);

    public:
        bool solve(
            const Coord&       dronePos,
            float              droneDir,
            float              droneSpeed,
            const Target&      target,
            float              currentTime,
            bool               alreadyApproaching,
            const DroneConfig& cfg,
            const AmmoParams&  ammo,
            Coord& outFirePos,
            Coord& outIntermediatePos,
            bool&  outHasIntermediate,
            float& outTotalTime,
            Coord& outPredictedTarget,
            Coord& outAimPoint) override;
};
