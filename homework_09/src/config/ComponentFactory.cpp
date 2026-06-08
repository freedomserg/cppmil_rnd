#include "config/ComponentFactory.h"
#include <memory>
#include "config/FileConfigLoader.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"

std::unique_ptr<IBallisticSolver> createSolver(SolverType type, const DroneConfig& cfg, const AmmoParams& ammo) {
    switch (type) {
        case SolverType::ANALYTICAL: return std::make_unique<AnalyticalSolver>(cfg, ammo);
        case SolverType::TABLE:      return std::make_unique<TableSolver>(cfg, ammo, "data/ballistic_table.txt");
    }
    return nullptr;
}

std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& filename) {
    switch (type) {
        case ProviderType::JSON: return std::make_unique<JsonTargetProvider>(filename);
    }
    return nullptr;
}

std::unique_ptr<IConfigLoader> createLoader(LoaderType type, const std::string& configFile, const std::string& ammoFile) {
    switch (type) {
        case LoaderType::FILE: return std::make_unique<FileConfigLoader>(configFile, ammoFile);
    }
    return nullptr;
}
