#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "Types.h"
#include <memory>
#include <string>
#include <utility>

class IBallisticSolverFactory {
public:
    virtual auto create() -> std::unique_ptr<IBallisticSolver> = 0;
    virtual ~IBallisticSolverFactory() = default;
};

class AnalyticalSolverFactory : public IBallisticSolverFactory {
    DroneConfig cfg_;
    AmmoParams  ammo_;
public:
    AnalyticalSolverFactory(DroneConfig cfg, AmmoParams ammo)
        : cfg_(std::move(cfg)), ammo_(std::move(ammo)) {}
    auto create() -> std::unique_ptr<IBallisticSolver> override;
};

class TableSolverFactory : public IBallisticSolverFactory {
    DroneConfig cfg_;
    AmmoParams  ammo_;
    std::string tablePath_;
public:
    TableSolverFactory(DroneConfig cfg, AmmoParams ammo, std::string tablePath)
        : cfg_(std::move(cfg)), ammo_(std::move(ammo)), tablePath_(std::move(tablePath)) {}
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
