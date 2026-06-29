#pragma once

#include "interfaces/ITargetProvider.h"
#include "Types.h"

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

class ThreadSafeTargetProvider : public ITargetProvider {
public:
    ThreadSafeTargetProvider(const std::string& filename, float arrayTimeStep, float timeScale);

    void run();                         // тіло потоку
    void start();                       // дозволити рух цілей
    void stop();                        // підняти стоп-прапорець
    [[nodiscard]] auto isThreadReady() const -> bool;

    [[nodiscard]] auto getTargetCount() const -> int override;
    [[nodiscard]] auto getTarget(int idx) const -> Target override;

private:
    static auto loadTrajectories(const std::string& filename) -> std::vector<std::vector<Coord>>;
    void advance();          // крок до наступного вузла + перерахунок
    void recompute();        // вимагає утримання mutex_

    mutable std::mutex mutex_;
    std::vector<std::vector<Coord>> traj_;
    std::vector<Target> targets_;
    int   nodeCount_     = 0;
    int   step_          = 0;
    float arrayTimeStep_ = 0.0f;
    float timeScale_     = 1.0f;

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> running_{true};
};
