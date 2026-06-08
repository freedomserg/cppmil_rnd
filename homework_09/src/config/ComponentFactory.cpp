#include "config/ComponentFactory.h"
#include <memory>
#include "config/FileConfigLoader.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"

std::unique_ptr<IBallisticSolver> AnalyticalSolverFactory::create() {
    return std::make_unique<AnalyticalSolver>(cfg_, ammo_);
}

std::unique_ptr<IBallisticSolver> TableSolverFactory::create() {
    return std::make_unique<TableSolver>(cfg_, ammo_, tablePath_);
}

std::unique_ptr<ITargetProvider> JsonFileProviderFactory::create() {
    return std::make_unique<JsonTargetProvider>(filename_);
}

std::unique_ptr<IConfigLoader> FileConfigLoaderFactory::create() {
    return std::make_unique<FileConfigLoader>(configFile_, ammoFile_);
}
