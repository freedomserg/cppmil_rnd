#pragma once

#include "interfaces/IBallisticSolver.h"

struct DroneConfig;
struct AmmoParams;
struct Coord;
struct Target;

// Спільна геометрія місії + solve(). Нащадки задають балістику через setBallistics().
class BaseBallisticSolver : public IBallisticSolver {
protected:
    // Нащадки задають балістику снаряда (час польоту + горизонтальна дистанція).
    void setBallistics(float ammoFlightTime, float hDist);

    void getIntermediateAndDropPoint(
        const Coord& dronePos, const Coord& targetPos,
        float hDist, float accelPath,
        Coord& outIntermediatePos, Coord& outFirePos, bool& outHasIntermediate);

    // прогноз = поточна позиція + швидкість * dt
    auto extrapolate(const Target& target, float dt) -> Coord;

    auto needStop(float desiredDir, float currentDir, float angleThreshold) -> bool;

    auto calcTimeOfFlight(
        Coord currentPos, const Coord& targetPos,
        float currentDir, float angleThreshold, float angularSpeed,
        float currentSpeed, float maxSpeed, float accelPath) -> float;

public:
    auto solve(
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
        Coord& outAimPoint) -> bool override;

private:
    float ammoFlightTime_ = 0.0f;
    float hDist_          = 0.0f;
    bool  ready_          = false;
};
