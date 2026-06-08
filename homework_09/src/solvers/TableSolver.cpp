#include "solvers/TableSolver.h"
#include "Types.h"
#include "Logger.h"

TableSolver::TableSolver(const DroneConfig& cfg, const AmmoParams& ammo,
                         const std::string& tablePath)
{
    if (!table_.load(tablePath)) {
        LOG("Error: failed to load ballistic table from " << tablePath);
        ready_ = false;
        return;
    }

    // Параметри пострілу фіксовані на всю місію → достатньо одного lookup.
    BallisticTable::Result r = table_.lookup(
        cfg.altitude, cfg.attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    ammoFlightTime_ = r.t;
    hDist_          = r.hDist;
    ready_          = true;

    LOG("Ammo flight time (table): " << ammoFlightTime_
        << " s, horizontal distance: " << hDist_ << " m");
}
