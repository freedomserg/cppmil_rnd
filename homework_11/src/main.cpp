#include "MissionPlanner.h"
#include "control/DroneController.h"
#include "gpio/GpioFactory.h"
#include "uart/UartLink.h"
#include "uart/UartTargetStore.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include "MathUtils.h"
#include "Logger.h"

#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

namespace {

struct Args {
    std::string uart      = "/tmp/ttyA";
    std::string gpiochip  = "gpiochip1";
    unsigned    startLine = 24;
    unsigned    dropLine  = 23;
    std::string solver    = "analytical";   // "analytical" | "table"
    std::string table     = "data/ballistic_table.txt";
};

auto parseArgs(int argc, char** argv) -> Args {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&](const std::string& def) -> std::string {
            return (i + 1 < argc) ? std::string(argv[++i]) : def;
        };
        if      (k == "--uart")       a.uart = next(a.uart);
        else if (k == "--gpiochip")   a.gpiochip = next(a.gpiochip);
        else if (k == "--start-line") a.startLine = static_cast<unsigned>(std::strtoul(next("24").c_str(), nullptr, 10));
        else if (k == "--drop-line")  a.dropLine  = static_cast<unsigned>(std::strtoul(next("23").c_str(), nullptr, 10));
        else if (k == "--solver")     a.solver = next(a.solver);
        else if (k == "--table")      a.table = next(a.table);
    }
    return a;
}

auto makeSolver(const Args& args, const DroneConfig& cfg, const AmmoParams& ammo)
    -> std::unique_ptr<IBallisticSolver> {
    if (args.solver == "table")
        return std::make_unique<TableSolver>(cfg, ammo, args.table);
    return std::make_unique<AnalyticalSolver>(cfg, ammo);
}

}  // namespace

auto main(int argc, char** argv) -> int {
    const Args args = parseArgs(argc, argv);

    UartLink uart;
    if (!uart.open(args.uart.c_str())) {
        LOG("Failed to open UART: " << args.uart);
        return 1;
    }

    auto gpio = makeGpio(args.gpiochip, args.startLine, args.dropLine);
    gpio->setStart(true);   // «готовий» → чекер починає симуляцію
    LOG("START raised on " << args.gpiochip << " line " << args.startLine
        << "; reading telemetry on " << args.uart);

    DroneConfig cfg{};
    AmmoParams  ammo{};
    UartTargetStore targets;
    std::unique_ptr<MissionPlanner>  planner;
    std::unique_ptr<DroneController> controller;
    bool     haveAmmo    = false;
    bool     haveCfg     = false;
    bool     dropped     = false;
    uint32_t lastClockMs = 0;

    UartHandlers h;
    h.onAmmo = [&](const dlink::AmmoCfg& a) {
        ammo.name = std::string(a.name, ::strnlen(a.name, sizeof a.name));
        ammo.mass = a.mass;
        ammo.drag = a.drag;
        ammo.lift = a.lift;
        cfg.hitRadius = a.hitRadius;
        targets.resize(a.nTargets);
        haveAmmo = true;
        LOG("AMMO: " << ammo.name << " m=" << ammo.mass << " d=" << ammo.drag
            << " l=" << ammo.lift << " hitR=" << cfg.hitRadius
            << " nTargets=" << static_cast<int>(a.nTargets));
    };
    h.onConfig = [&](const dlink::DroneCfg& c) {
        cfg.attackSpeed   = c.attackSpeed;
        cfg.accelPath     = c.accelerationPath;
        cfg.angularSpeed  = c.angularSpeed;
        cfg.turnThreshold = c.turnThreshold;
        cfg.simTimeStep   = c.timeStep;
        cfg.timeScale     = c.timeScale;
        cfg.solverType    = args.solver;
        haveCfg = true;
        LOG("CONFIG: V=" << cfg.attackSpeed << " accelPath=" << cfg.accelPath
            << " w=" << cfg.angularSpeed << " turnThr=" << cfg.turnThreshold
            << " dt=" << cfg.simTimeStep);
    };
    h.onTarget = [&](const dlink::TargetPos& tp) {
        targets.update(tp.id, Coord{tp.x, tp.y}, lastClockMs);
    };
    h.onResult = [&](const dlink::Result& r) {
        LOG((r.hit ? "RESULT: HIT" : "RESULT: MISS") << " miss=" << r.miss_m
            << "m target=" << static_cast<int>(r.targetId) << " t=" << r.drop_t_ms << "ms");
    };
    h.onTelemetry = [&](const dlink::Telemetry& t) {
        lastClockMs = t.t_ms;

        if (!planner && haveAmmo && haveCfg) {
            cfg.altitude = t.z;                    // висота є лише в телеметрії
            planner    = std::make_unique<MissionPlanner>(makeSolver(args, cfg, ammo), cfg, ammo);
            controller = std::make_unique<DroneController>(cfg);
            LOG("Solver=" << args.solver << " built at altitude " << cfg.altitude);
        }

        const DroneTelemetry tel{
            .pos               = {t.x, t.y},
            .speed             = {t.vx, t.vy},
            .direction         = t.dir,
            .timeSecSinceStart = static_cast<float>(t.t_ms) / 1000.0f,
        };

        if (dropped) { uart.sendControl(0.0f, 0.0f); return; }

        if (planner && targets.allSeen()) {
            const PlanResult pr = planner->plan(tel, targets);
            if (pr.valid) {
                const auto out = controller->toControl(pr.command, length(tel.speed));
                uart.sendControl(out.accel, out.turnRate);
            } else {
                uart.sendControl(0.0f, 0.0f);
            }
            if (pr.drop) {
                gpio->pulseDrop();
                dropped = true;
                LOG("DROP at pos (" << tel.pos.x << ", " << tel.pos.y << ")");
            }
        } else {
            uart.sendControl(0.0f, 0.0f);          // тримаємо дрон живим до готовності
        }
    };
    uart.setHandlers(std::move(h));

    constexpr int        kGraceFrames = 50;
    constexpr useconds_t kIdleUs      = 500;
    int graceFrames = 0;
    while (true) {
        const int n = uart.poll();
        if (dropped && ++graceFrames > kGraceFrames) break;
        if (n <= 0) ::usleep(kIdleUs);
    }

    LOG("Mission loop finished.");
    return 0;
}
