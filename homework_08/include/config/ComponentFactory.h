#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include "solvers/AnalyticalSolver.h"
#include <string>

struct DroneConfig;
struct AmmoParams;

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

IBallisticSolver* createSolver(SolverType type,
    const DroneConfig& cfg, const AmmoParams& ammo);

ITargetProvider* createProvider(ProviderType type, const std::string& filename);

IConfigLoader* createLoader(LoaderType type,
    const std::string& configFile, const std::string& ammoFile);
