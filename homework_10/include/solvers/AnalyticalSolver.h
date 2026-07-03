#pragma once

#include "solvers/BaseBallisticSolver.h"

struct DroneConfig;
struct AmmoParams;

class AnalyticalSolver : public BaseBallisticSolver {
private:
    auto calcAmmoDropTime(const DroneConfig& cfg, const AmmoParams& ammo, float& outT) -> bool;
    auto calcAmmoHDistance(const DroneConfig& cfg, const AmmoParams& ammo, float t) -> float;

public:
    AnalyticalSolver(const DroneConfig& cfg, const AmmoParams& ammo);
};
