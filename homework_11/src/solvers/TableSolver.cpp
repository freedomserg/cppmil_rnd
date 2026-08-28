#include "solvers/TableSolver.h"
#include "Types.h"
#include "Logger.h"

TableSolver::TableSolver(const DroneConfig& cfg, const AmmoParams& ammo,
                         const std::string& tablePath) {
    if (!table_.load(tablePath)) {
        LOG("Error: failed to load ballistic table from " << tablePath);
        return;
    }

    BallisticTable::Result r = table_.lookup(
        cfg.altitude, cfg.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    setBallistics(r.t, r.hDist);

    LOG("Ammo flight time (table): " << r.t
        << " s, horizontal distance: " << r.hDist << " m");
}
