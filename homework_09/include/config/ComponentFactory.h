#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include "solvers/AnalyticalSolver.h"
#include <string>
#include <memory>

struct DroneConfig;
struct AmmoParams;

enum class SolverType   { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

std::unique_ptr<IBallisticSolver> createSolver(SolverType type,
    const DroneConfig& cfg, const AmmoParams& ammo);

std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& filename);

std::unique_ptr<IConfigLoader> createLoader(LoaderType type,
    const std::string& configFile, const std::string& ammoFile);
