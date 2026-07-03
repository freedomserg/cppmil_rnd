#pragma once

#include "Types.h"
#include "concurrency/ThreadSafeQueue.h"

#include <atomic>
#include <mutex>

class DronePhysics {
public:
    explicit DronePhysics(const DroneConfig& cfg);

    void run();                         // тіло потоку
    void start();                       // дозволити інтегрування
    void stop();                        // підняти стоп-прапорець
    [[nodiscard]] auto isThreadReady() const -> bool;

    void pushCommand(const DroneCommand& cmd);
    [[nodiscard]] auto getTelemetry() const -> DroneTelemetry;

private:
    void step(float dt);

    mutable std::mutex mutex_;
    Coord pos_{};
    float direction_         = 0.0f;
    float speed_             = 0.0f;
    float timeSecSinceStart_ = 0.0f;

    DroneCommand current_{};
    ThreadSafeQueue<DroneCommand> commands_;

    float accel_           = 0.0f;
    float attackSpeed_     = 0.0f;
    float physicsTimeStep_ = 0.0f;
    float timeScale_       = 1.0f;

    std::atomic<bool> ready_{false};
    std::atomic<bool> started_{false};
    std::atomic<bool> running_{true};
};
