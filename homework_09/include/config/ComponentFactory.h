#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include "Types.h"
#include <string>
#include <memory>

class IBallisticSolverFactory {
public:
    virtual std::unique_ptr<IBallisticSolver> create() = 0;
    virtual ~IBallisticSolverFactory() = default;
};

class AnalyticalSolverFactory : public IBallisticSolverFactory {
    DroneConfig cfg_;
    AmmoParams  ammo_;
public:
    AnalyticalSolverFactory(const DroneConfig& cfg, const AmmoParams& ammo)
        : cfg_(cfg), ammo_(ammo) {}
    std::unique_ptr<IBallisticSolver> create() override;
};

class TableSolverFactory : public IBallisticSolverFactory {
    DroneConfig cfg_;
    AmmoParams  ammo_;
    std::string tablePath_;
public:
    TableSolverFactory(const DroneConfig& cfg, const AmmoParams& ammo, std::string tablePath)
        : cfg_(cfg), ammo_(ammo), tablePath_(std::move(tablePath)) {}
    std::unique_ptr<IBallisticSolver> create() override;
};

class ITargetProviderFactory {
public:
    virtual std::unique_ptr<ITargetProvider> create() = 0;
    virtual ~ITargetProviderFactory() = default;
};

class JsonFileProviderFactory : public ITargetProviderFactory {
    std::string filename_;
public:
    JsonFileProviderFactory(std::string filename) : filename_(std::move(filename)) {}
    std::unique_ptr<ITargetProvider> create() override;
};

class IConfigLoaderFactory {
public:
    virtual std::unique_ptr<IConfigLoader> create() = 0;
    virtual ~IConfigLoaderFactory() = default;
};

class FileConfigLoaderFactory : public IConfigLoaderFactory {
    std::string configFile_;
    std::string ammoFile_;
public:
    FileConfigLoaderFactory(std::string configFile, std::string ammoFile)
        : configFile_(std::move(configFile)), ammoFile_(std::move(ammoFile)) {}
    std::unique_ptr<IConfigLoader> create() override;
};
