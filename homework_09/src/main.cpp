#include <memory>
#include "config/ComponentFactory.h"
#include "MissionProcessor.h"
#include "Logger.h"

int main() {
    FileConfigLoaderFactory loaderFactory("data/config.json", "data/ammo.json");
    std::unique_ptr<IConfigLoader> loader = loaderFactory.create();
    if (!loader->load()) {
        LOG("Failed to load configuration");
        return 1;
    }

    DroneConfig cfg = loader->getConfig();
    LOG("Drone and simulation config:");
    LOG("  Initial position: (" << cfg.initialPos.x << ", " << cfg.initialPos.y << ")");
    LOG("  Altitude: "          << cfg.altitude      << " m");
    LOG("  Initial direction: " << cfg.initialDir    << " rad");
    LOG("  Attack speed: "      << cfg.attackSpeed   << " m/s");
    LOG("  Acceleration path: " << cfg.accelPath     << " m");
    LOG("  Ammo type: "         << cfg.ammoName);
    LOG("  Array time step: "   << cfg.arrayTimeStep << " s");
    LOG("  Sim time step: "     << cfg.simTimeStep   << " s");
    LOG("  Hit radius: "        << cfg.hitRadius     << " m");
    LOG("  Angular speed: "     << cfg.angularSpeed  << " rad/s");
    LOG("  Turn threshold: "    << cfg.turnThreshold << " rad");

    JsonFileProviderFactory providerFactory("data/targets.json");
    std::unique_ptr<ITargetProvider> provider = providerFactory.create();
    if (provider->getTargetCount() == 0) {
        LOG("Failed to load targets from data/targets.json");
        return 1;
    }

    std::unique_ptr<IBallisticSolverFactory> solverFactory;
    if (cfg.solverType == "table") {
        solverFactory = std::make_unique<TableSolverFactory>(
            loader->getConfig(), loader->getAmmoParams(), "data/ballistic_table.txt");
    } else {
        if (cfg.solverType != "analytical")
            LOG("Unknown solver '" << cfg.solverType << "', falling back to analytical");
        solverFactory = std::make_unique<AnalyticalSolverFactory>(
            loader->getConfig(), loader->getAmmoParams());
    }
    LOG("Using solver: " << cfg.solverType);

    std::unique_ptr<IBallisticSolver> solver = solverFactory->create();

    MissionProcessor mission(std::move(provider), std::move(solver), std::move(loader));
    if (!mission.init()) {
        LOG("Failed to initialize mission");
        return 1;
    }

    while (mission.hasNext())
        mission.step();

    if (!mission.writeOutput("simulation.json"))
        LOG("Error writing output to file");

    LOG("Simulation completed. Recorded steps: " << mission.getRecordedSteps());
    if (mission.getRecordedSteps() >= 10000)
        LOG("Warning: simulation reached MAX_STEPS without drop");

    return 0;
}
