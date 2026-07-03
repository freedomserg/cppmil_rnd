#include "config/ComponentFactory.h"
#include "config/FileConfigLoader.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include <memory>

auto AnalyticalSolverFactory::create() -> std::unique_ptr<IBallisticSolver> {
    return std::make_unique<AnalyticalSolver>(cfg_, ammo_);
}

auto TableSolverFactory::create() -> std::unique_ptr<IBallisticSolver> {
    return std::make_unique<TableSolver>(cfg_, ammo_, tablePath_);
}

auto FileConfigLoaderFactory::create() -> std::unique_ptr<IConfigLoader> {
    return std::make_unique<FileConfigLoader>(configFile_, ammoFile_);
}
