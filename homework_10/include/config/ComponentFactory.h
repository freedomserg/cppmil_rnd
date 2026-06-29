#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "Types.h"
#include <memory>
#include <string>

class IBallisticSolverFactory {
public:
    virtual auto create() -> std::unique_ptr<IBallisticSolver> = 0;
    virtual ~IBallisticSolverFactory() = default;
};

class AnalyticalSolverFactory : public IBallisticSolverFactory {
    DroneConfig cfg_;
    AmmoParams  ammo_;
public:
    AnalyticalSolverFactory(const DroneConfig& cfg, const AmmoParams& ammo)
        : cfg_(cfg), ammo_(ammo) {}
    auto create() -> std::unique_ptr<IBallisticSolver> override;
};

class TableSolverFactory : public IBallisticSolverFactory {
    DroneConfig cfg_;
    AmmoParams  ammo_;
    std::string tablePath_;
public:
    TableSolverFactory(const DroneConfig& cfg, const AmmoParams& ammo, std::string tablePath)
        : cfg_(cfg), ammo_(ammo), tablePath_(std::move(tablePath)) {}
    auto create() -> std::unique_ptr<IBallisticSolver> override;
};

class IConfigLoaderFactory {
public:
    virtual auto create() -> std::unique_ptr<IConfigLoader> = 0;
    virtual ~IConfigLoaderFactory() = default;
};

class FileConfigLoaderFactory : public IConfigLoaderFactory {
    std::string configFile_;
    std::string ammoFile_;
public:
    FileConfigLoaderFactory(std::string configFile, std::string ammoFile)
        : configFile_(std::move(configFile)), ammoFile_(std::move(ammoFile)) {}
    auto create() -> std::unique_ptr<IConfigLoader> override;
};
