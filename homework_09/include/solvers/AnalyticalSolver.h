#pragma once

#include "solvers/BaseBallisticSolver.h"

struct DroneConfig;
struct AmmoParams;

class AnalyticalSolver : public BaseBallisticSolver {
    private:
        // Розраховує час падіння боєприпасу з висоти cfg.altitude до землі.
        // Рівняння руху — кубічне, вирішується методом Кардано через arccos.
        bool calcAmmoDropTime(const DroneConfig& cfg, const AmmoParams& ammo, float& outT);

        // Горизонтальна відстань яку пролетить боєприпас за час t.
        float calcAmmoHDistance(const DroneConfig& cfg, const AmmoParams& ammo, float t);

    public:
        AnalyticalSolver(const DroneConfig& cfg, const AmmoParams& ammo);
};
