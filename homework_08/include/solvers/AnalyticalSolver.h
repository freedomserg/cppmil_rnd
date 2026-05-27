#pragma once

#include "interfaces/IBallisticSolver.h"

struct DroneConfig;
struct AmmoParams;
struct Coord;
struct Target;

class AnalyticalSolver : public IBallisticSolver {
    private:
        float ammoFlightTime_ = 0.0f;
        float hDist_          = 0.0f;
        bool  ready_          = false;

        // Розраховує час падіння боєприпасу з висоти cfg.altitude до землі.
        // Рівняння руху — кубічне; вирішується методом Кардано через arccos.
        bool calcAmmoDropTime(const DroneConfig& cfg, const AmmoParams& ammo, float& outT);

        // Горизонтальна відстань яку пролетить боєприпас за час t.
        float calcAmmoHDistance(const DroneConfig& cfg, const AmmoParams& ammo, float t);

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
            AnalyticalSolver(const DroneConfig& cfg, const AmmoParams& ammo);

            bool solve(
                const Coord&       dronePos,
                float              droneDir,
                float              droneSpeed,
                const Target&      target,
                float              currentTime,
                bool               alreadyApproaching,
                const DroneConfig& cfg,
                const AmmoParams&,
                Coord& outFirePos,
                Coord& outIntermediatePos,
                bool&  outHasIntermediate,
                float& outTotalTime,
                Coord& outPredictedTarget,
                Coord& outAimPoint) override;



};