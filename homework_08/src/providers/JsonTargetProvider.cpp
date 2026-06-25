#include "providers/JsonTargetProvider.h"
#include "Logger.h"
#include "json/json.hpp"
#include <fstream>

using json = nlohmann::json;

// ── Private helper ──────────────────────────────────────────────────────────

std::vector<std::vector<Coord>> JsonTargetProvider::loadTargetsCoordinatesVector(
    const std::string& filename)
{
    std::fstream f(filename, std::ios::in);
    if (!f) { LOG("Error opening targets file: " << filename); return {}; }

    json data = json::parse(f);
    f.close();

    const int steps = data["timeSteps"];
    std::vector<std::vector<Coord>> vec;
    vec.reserve(data["targetCount"]);

    for (const auto& target : data["targets"]) {
        std::vector<Coord> row;
        row.reserve(steps);
        for (const auto& pos : target["positions"])
            row.push_back({ pos["x"].get<float>(), pos["y"].get<float>() });
        vec.push_back(std::move(row));
    }

    return vec;
}

// ── Constructor ─────────────────────────────────────────────────────────────

JsonTargetProvider::JsonTargetProvider(const std::string& filename) {
    vec_ = loadTargetsCoordinatesVector(filename);
}

// ── ITargetProvider ─────────────────────────────────────────────────────────

int JsonTargetProvider::getTargetCount() {
    return static_cast<int>(vec_.size());
}

Target JsonTargetProvider::getTarget(int idx) {
    return { vec_[idx].data(), static_cast<int>(vec_[idx].size()) };
}
