#include "config/ComponentFactory.h"
#include "config/FileConfigLoader.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"

IBallisticSolver* createSolver(SolverType type, const DroneConfig& cfg, const AmmoParams& ammo) {
    switch (type) {
        case SolverType::ANALYTICAL: return new AnalyticalSolver(cfg, ammo);
    }
    return nullptr;
}

ITargetProvider* createProvider(ProviderType type, const std::string& filename) {
    switch (type) {
        case ProviderType::JSON: return new JsonTargetProvider(filename);
    }
    return nullptr;
}

IConfigLoader* createLoader(LoaderType type, const std::string& configFile, const std::string& ammoFile) {
    switch (type) {
        case LoaderType::FILE: return new FileConfigLoader(configFile, ammoFile);
    }
    return nullptr;
}
