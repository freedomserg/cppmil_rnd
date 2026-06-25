#include "config/ComponentFactory.h"
#include "MissionProcessor.h"
#include "Logger.h"

int main() {
    IConfigLoader* loader = createLoader(LoaderType::FILE, "data/config.json", "data/ammo.json");
    if (!loader->load()) {
        LOG("Failed to load configuration");
        delete loader;
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

    ITargetProvider* provider = createProvider(ProviderType::JSON, "data/targets.json");
    if (provider->getTargetCount() == 0) {
        LOG("Failed to load targets from data/targets.json");
        delete provider;
        delete loader;
        return 1;
    }

    IBallisticSolver* solver = createSolver(SolverType::ANALYTICAL,
        loader->getConfig(), loader->getAmmoParams());

    MissionProcessor mission(provider, solver, loader);
    if (!mission.init()) {
        LOG("Failed to initialize mission");
        delete solver;
        delete provider;
        delete loader;
        return 1;
    }

    while (mission.hasNext())
        mission.step();

    if (!mission.writeOutput("simulation.json"))
        LOG("Error writing output to file");

    LOG("Simulation completed. Recorded steps: " << mission.getRecordedSteps());
    if (mission.getRecordedSteps() >= 10000)
        LOG("Warning: simulation reached MAX_STEPS without drop");

    delete solver;
    delete provider;
    delete loader;
    return 0;
}
