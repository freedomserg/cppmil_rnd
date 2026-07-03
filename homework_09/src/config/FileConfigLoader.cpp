#include "config/FileConfigLoader.h"
#include "Logger.h"
#include "json/json.hpp"
#include <fstream>

using json = nlohmann::json;

// ── Private helpers ─────────────────────────────────────────────────────────

DroneConfig FileConfigLoader::loadDroneConfig(const std::string& filename, bool& outLoaded) {
    std::fstream f(filename, std::ios::in);
    if (!f) { LOG("Error opening input file: " << filename); outLoaded = false; return {}; }

    json data = json::parse(f);
    f.close();

    DroneConfig cfg = {
        .initialPos    = {data["drone"]["position"]["x"].get<float>(),
                          data["drone"]["position"]["y"].get<float>()},
        .altitude      = data["drone"]["altitude"].get<float>(),
        .initialDir    = data["drone"]["initialDirection"].get<float>(),
        .attackSpeed   = data["drone"]["attackSpeed"].get<float>(),
        .accelPath     = data["drone"]["accelerationPath"].get<float>(),
        .ammoName = data["ammo"].get<std::string>(),
        .arrayTimeStep = data["targetArrayTimeStep"].get<float>(),
        .simTimeStep   = data["simulation"]["timeStep"].get<float>(),
        .hitRadius     = data["simulation"]["hitRadius"].get<float>(),
        .angularSpeed  = data["drone"]["angularSpeed"].get<float>(),
        .turnThreshold = data["drone"]["turnThreshold"].get<float>(),
        .solverType    = data.value("solver", std::string("analytical")),
    };
    outLoaded = true;
    return cfg;
}

std::vector<AmmoParams> FileConfigLoader::loadAmmoParamsArray(const std::string& filename) {
    std::fstream f(filename, std::ios::in);
    if (!f) { LOG("Error opening ammo file: " << filename); return {}; }

    json data = json::parse(f);
    f.close();

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

// ── Constructor ─────────────────────────────────────────────────────────────

FileConfigLoader::FileConfigLoader(const std::string& configFile, const std::string& ammoFile)
    : configFile_(configFile), ammoFile_(ammoFile) {}

// ── IConfigLoader ───────────────────────────────────────────────────────────

bool FileConfigLoader::load() {
    bool ok = false;
    cfg_ = loadDroneConfig(configFile_, ok);
    if (!ok) return false;

    auto arr = loadAmmoParamsArray(ammoFile_);
    if (arr.empty()) return false;

    bool found = false;
    for (const auto& ammo : arr) {
        if (cfg_.ammoName == ammo.name) {
            ammo_ = ammo;
            found = true;
            break;
        }
    }

    if (!found) { LOG("Error: unknown ammo type: " << cfg_.ammoName); return false; }

    constexpr float PI = std::numbers::pi_v<float>;
    if (std::fabs(cfg_.initialDir)    > PI)    { LOG("Error: initialDir out of [-π,π]");     return false; }
    if (cfg_.attackSpeed   <= 0.0f) { LOG("Error: attackSpeed must be > 0");   return false; }
    if (cfg_.accelPath     <= 0.0f) { LOG("Error: accelPath must be > 0");     return false; }
    if (cfg_.arrayTimeStep <= 0.0f) { LOG("Error: arrayTimeStep must be > 0"); return false; }
    if (cfg_.simTimeStep   <= 0.0f) { LOG("Error: simTimeStep must be > 0");   return false; }
    if (cfg_.hitRadius     <= 0.0f) { LOG("Error: hitRadius must be > 0");     return false; }
    if (cfg_.angularSpeed  <= 0.0f) { LOG("Error: angularSpeed must be > 0");  return false; }
    if (std::fabs(cfg_.turnThreshold) > PI) { LOG("Error: turnThreshold out of [-π,π]"); return false; }

    return true;
}

DroneConfig FileConfigLoader::getConfig()     const { return cfg_; }
AmmoParams  FileConfigLoader::getAmmoParams() const { return ammo_; }
