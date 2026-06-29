#include "providers/ThreadSafeTargetProvider.h"
#include "Logger.h"
#include <json.hpp>

#include <chrono>
#include <fstream>
#include <thread>

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(
    const std::string& filename, float arrayTimeStep, float timeScale)
    : traj_(loadTrajectories(filename)),
      nodeCount_(traj_.empty() ? 0 : static_cast<int>(traj_[0].size())),
      arrayTimeStep_(arrayTimeStep),
      timeScale_(timeScale) {
    targets_.resize(traj_.size());
    recompute();
}

auto ThreadSafeTargetProvider::loadTrajectories(const std::string& filename)
    -> std::vector<std::vector<Coord>> {
    std::ifstream f(filename);
    if (!f) { LOG("Error opening targets file: " << filename); return {}; }

    json data = json::parse(f);
    std::vector<std::vector<Coord>> vec;
    vec.reserve(data["targetCount"].get<size_t>());
    for (const auto& target : data["targets"]) {
        std::vector<Coord> row;
        row.reserve(target["positions"].size());
        for (const auto& p : target["positions"])
            row.push_back({p["x"].get<float>(), p["y"].get<float>()});
        vec.push_back(std::move(row));
    }
    return vec;
}

void ThreadSafeTargetProvider::recompute() {
    if (nodeCount_ == 0) return;
    int next = (step_ + 1) % nodeCount_;
    for (size_t i = 0; i < traj_.size(); ++i) {
        const Coord pos  = traj_[i][step_];
        const Coord velocity = (traj_[i][next] - pos) / arrayTimeStep_;
        targets_[i] = {pos, velocity};
    }
}

void ThreadSafeTargetProvider::advance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (nodeCount_ == 0) return;
    step_ = (step_ + 1) % nodeCount_;
    recompute();
}

void ThreadSafeTargetProvider::start() { started_ = true; }
void ThreadSafeTargetProvider::stop()  { running_ = false; }
auto ThreadSafeTargetProvider::isThreadReady() const -> bool { return ready_; }

void ThreadSafeTargetProvider::run() {
    ready_ = true;
    while (!started_ && running_)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    while (running_) {
        advance();
        std::this_thread::sleep_for(std::chrono::duration<float>(arrayTimeStep_ / timeScale_));
    }
}

auto ThreadSafeTargetProvider::getTargetCount() const -> int {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(targets_.size());
}

auto ThreadSafeTargetProvider::getTarget(int idx) const -> Target {
    std::lock_guard<std::mutex> lock(mutex_);
    return targets_[idx];
}
