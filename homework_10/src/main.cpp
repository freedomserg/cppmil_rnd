#include "config/ComponentFactory.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "physics/DronePhysics.h"
#include "MissionProcessor.h"
#include "Logger.h"

#include <chrono>
#include <memory>
#include <thread>

constexpr int kMaxSteps = 10000;

auto main() -> int {
    FileConfigLoaderFactory loaderFactory("data/config.json", "data/ammo.json");
    std::unique_ptr<IConfigLoader> loader = loaderFactory.create();
    if (!loader || !loader->load()) {
        LOG("Failed to load configuration");
        return 1;
    }

    const DroneConfig cfg  = loader->getConfig();
    const AmmoParams  ammo = loader->getAmmoParams();
    LOG("Config: solver=" << cfg.solverType
        << ", simTimeStep=" << cfg.simTimeStep
        << ", physicsTimeStep=" << cfg.physicsTimeStep
        << ", arrayTimeStep=" << cfg.arrayTimeStep
        << ", timeScale=" << cfg.timeScale);

    auto provider = std::make_unique<ThreadSafeTargetProvider>(
        "data/targets.json", cfg.arrayTimeStep, cfg.timeScale);
    if (provider->getTargetCount() == 0) {
        LOG("Failed to load targets from data/targets.json");
        return 1;
    }

    auto physics = std::make_unique<DronePhysics>(cfg);

    std::unique_ptr<IBallisticSolverFactory> solverFactory;
    if (cfg.solverType == "table") {
        solverFactory = std::make_unique<TableSolverFactory>(cfg, ammo, "data/ballistic_table.txt");
    } else {
        if (cfg.solverType != "analytical")
            LOG("Unknown solver '" << cfg.solverType << "', falling back to analytical");
        solverFactory = std::make_unique<AnalyticalSolverFactory>(cfg, ammo);
    }
    std::unique_ptr<IBallisticSolver> solver = solverFactory->create();
    if (!solver) { LOG("Failed to create solver"); return 1; }

    auto mission = std::make_unique<MissionProcessor>(
        provider.get(), physics.get(), std::move(solver), cfg, ammo);

    std::thread providerThread(&ThreadSafeTargetProvider::run, provider.get());
    std::thread physicsThread(&DronePhysics::run, physics.get());
    std::thread missionThread(&MissionProcessor::run, mission.get());

    while (!provider->isThreadReady() || !physics->isThreadReady() || !mission->isThreadReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    provider->start();
    physics->start();
    mission->start();

    missionThread.join();   // чекаємо лише на місію

    physics->stop();
    provider->stop();
    physicsThread.join();
    providerThread.join();

    if (!mission->writeOutput("simulation.json"))
        LOG("Error writing output to file");

    LOG("Simulation completed. Recorded steps: " << mission->getRecordedSteps());
    if (mission->getRecordedSteps() >= kMaxSteps)
        LOG("Warning: simulation reached MAX_STEPS without drop");

    return 0;
}
