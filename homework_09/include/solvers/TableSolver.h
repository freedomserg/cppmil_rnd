#pragma once

#include "solvers/BaseBallisticSolver.h"
#include "solvers/BallisticTable.h"
#include <string>

struct DroneConfig;
struct AmmoParams;

// Балістичний солвер на основі таблиці: час польоту та горизонтальну
// дистанцію бере з 5-вимірної таблиці (з інтерполяцією) замість формул.
// Решта логіки (геометрія місії) успадкована від BaseBallisticSolver.
class TableSolver : public BaseBallisticSolver {
    private:
        BallisticTable table_;

    public:
        TableSolver(const DroneConfig& cfg, const AmmoParams& ammo,
                    const std::string& tablePath);
};
