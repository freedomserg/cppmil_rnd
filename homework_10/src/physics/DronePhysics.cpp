#include "physics/DronePhysics.h"
#include "MathUtils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

DronePhysics::DronePhysics(const DroneConfig& cfg)
    : pos_(cfg.initialPos),
      direction_(cfg.initialDir),
      accel_(calcAccel(cfg.attackSpeed, cfg.accelPath)),
      attackSpeed_(cfg.attackSpeed),
      physicsTimeStep_(cfg.physicsTimeStep),
      timeScale_(cfg.timeScale) {}

void DronePhysics::pushCommand(const DroneCommand& cmd) { commands_.push(cmd); }

void DronePhysics::start() { started_ = true; }
void DronePhysics::stop()  { running_ = false; }
auto DronePhysics::isThreadReady() const -> bool { return ready_; }

void DronePhysics::run() {
    ready_ = true;
    while (!started_ && running_)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    const float dt = physicsTimeStep_;
    while (running_) {
        step(dt);
        std::this_thread::sleep_for(std::chrono::duration<float>(dt / timeScale_));
    }
}

void DronePhysics::step(float dt) {
    while (auto cmd = commands_.tryPop()) current_ = *cmd;

    std::lock_guard<std::mutex> lock(mutex_);
    direction_ = normalizeAngle(direction_ + current_.angleSpeed * dt);

    switch (current_.state) {
        case DroneState::Stopped:      speed_ = 0.0f; break;
        case DroneState::Accelerating: speed_ = std::min(attackSpeed_, speed_ + accel_ * dt); break;
        case DroneState::Decelerating: speed_ = std::max(0.0f, speed_ - accel_ * dt); break;
        case DroneState::Moving:       speed_ = attackSpeed_; break;
        case DroneState::Turning:      break;
    }

    pos_ = pos_ + Coord{std::cos(direction_), std::sin(direction_)} * (speed_ * dt);
    timeSecSinceStart_ += dt;
}

auto DronePhysics::getTelemetry() const -> DroneTelemetry {
    std::lock_guard<std::mutex> lock(mutex_);
    Coord velocity = Coord{std::cos(direction_), std::sin(direction_)} * speed_;
    return {pos_, velocity, direction_, timeSecSinceStart_};
}
