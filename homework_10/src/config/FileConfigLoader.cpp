#include "config/FileConfigLoader.h"
#include "Logger.h"
#include <json.hpp>
#include <cmath>
#include <fstream>
#include <numbers>

using json = nlohmann::json;

constexpr float kDefaultPhysicsTimeStep = 0.01f;
constexpr float kDefaultTimeScale       = 10.0f;

auto FileConfigLoader::loadDroneConfig(const std::string& filename, bool& outLoaded) -> DroneConfig {
    std::ifstream f(filename);
    if (!f) { LOG("Error opening input file: " << filename); outLoaded = false; return {}; }

    json data = json::parse(f);
    const auto& sim = data["simulation"];

    DroneConfig cfg = {
        .initialPos    = {data["drone"]["position"]["x"].get<float>(),
                          data["drone"]["position"]["y"].get<float>()},
        .altitude      = data["drone"]["altitude"].get<float>(),
        .initialDir    = data["drone"]["initialDirection"].get<float>(),
        .attackSpeed   = data["drone"]["attackSpeed"].get<float>(),
        .accelPath     = data["drone"]["accelerationPath"].get<float>(),
        .ammoName      = data["ammo"].get<std::string>(),
        .arrayTimeStep   = data["targetArrayTimeStep"].get<float>(),
        .simTimeStep     = sim["timeStep"].get<float>(),
        .physicsTimeStep = sim.value("physicsTimeStep", kDefaultPhysicsTimeStep),
        .timeScale       = sim.value("timeScale", kDefaultTimeScale),
        .hitRadius     = sim["hitRadius"].get<float>(),
        .angularSpeed  = data["drone"]["angularSpeed"].get<float>(),
        .turnThreshold = data["drone"]["turnThreshold"].get<float>(),
        .solverType    = data.value("solver", std::string("analytical")),
    };
    outLoaded = true;
    return cfg;
}

auto FileConfigLoader::loadAmmoParamsArray(const std::string& filename) -> std::vector<AmmoParams> {
    std::ifstream f(filename);
    if (!f) { LOG("Error opening ammo file: " << filename); return {}; }

    json data = json::parse(f);
    std::vector<AmmoParams> v;
    v.reserve(data.size());
    for (const auto& item : data) {
        v.push_back({
            .name = item["name"].get<std::string>(),
            .mass = item["mass"].get<float>(),
            .drag = item["drag"].get<float>(),
            .lift = item["lift"].get<float>(),
        });
    }
    return v;
}

FileConfigLoader::FileConfigLoader(const std::string& configFile, const std::string& ammoFile)
    : configFile_(configFile), ammoFile_(ammoFile) {}

auto FileConfigLoader::load() -> bool {
    bool ok = false;
    cfg_ = loadDroneConfig(configFile_, ok);
    if (!ok) return false;

    auto arr = loadAmmoParamsArray(ammoFile_);
    if (arr.empty()) return false;

    bool found = false;
    for (const auto& ammo : arr) {
        if (cfg_.ammoName == ammo.name) { ammo_ = ammo; found = true; break; }
    }
    if (!found) { LOG("Error: unknown ammo type: " << cfg_.ammoName); return false; }

    constexpr float PI = std::numbers::pi_v<float>;
    if (std::fabs(cfg_.initialDir)    > PI)   { LOG("Error: initialDir out of [-pi,pi]");      return false; }
    if (cfg_.attackSpeed     <= 0.0f) { LOG("Error: attackSpeed must be > 0");     return false; }
    if (cfg_.accelPath       <= 0.0f) { LOG("Error: accelPath must be > 0");       return false; }
    if (cfg_.arrayTimeStep   <= 0.0f) { LOG("Error: arrayTimeStep must be > 0");   return false; }
    if (cfg_.simTimeStep     <= 0.0f) { LOG("Error: simTimeStep must be > 0");     return false; }
    if (cfg_.physicsTimeStep <= 0.0f) { LOG("Error: physicsTimeStep must be > 0"); return false; }
    if (cfg_.timeScale       <= 0.0f) { LOG("Error: timeScale must be > 0");       return false; }
    if (cfg_.hitRadius       <= 0.0f) { LOG("Error: hitRadius must be > 0");       return false; }
    if (cfg_.angularSpeed    <= 0.0f) { LOG("Error: angularSpeed must be > 0");    return false; }
    if (std::fabs(cfg_.turnThreshold) > PI) { LOG("Error: turnThreshold out of [-pi,pi]"); return false; }

    return true;
}

auto FileConfigLoader::getConfig() const -> DroneConfig { return cfg_; }
auto FileConfigLoader::getAmmoParams() const -> AmmoParams { return ammo_; }
