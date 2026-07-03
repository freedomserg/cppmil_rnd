#pragma once

#include <string>
#include <vector>

// Data-only header: математика й важкі includes — у MathUtils.

struct Coord {
    float x{};
    float y{};

    auto operator+(const Coord& o) const -> Coord { return {x + o.x, y + o.y}; }
    auto operator-(const Coord& o) const -> Coord { return {x - o.x, y - o.y}; }
    auto operator*(float s) const -> Coord { return {x * s, y * s}; }
    auto operator/(float s) const -> Coord {
        if (s < 1e-6f && s > -1e-6f) return {0.0f, 0.0f};
        return {x / s, y / s};
    }
};

enum class DroneState : int {
    Stopped      = 0,
    Accelerating = 1,
    Decelerating = 2,
    Turning      = 3,
    Moving       = 4,
};

struct DroneCommand {
    DroneState state{DroneState::Stopped};
    float      angleSpeed{};
};

// direction — курс, валідний і за speed == 0 (Turning/Stopped).
struct DroneTelemetry {
    Coord pos{};
    Coord speed{};
    float direction{};
    float timeSecSinceStart{};
};

struct AmmoParams {
    std::string name;
    float mass{};
    float drag{};
    float lift{};
};

struct DroneConfig {
    Coord initialPos{};
    float altitude{};
    float initialDir{};
    float attackSpeed{};
    float accelPath{};
    std::string ammoName;
    float arrayTimeStep{};
    float simTimeStep{};
    float physicsTimeStep{};
    float timeScale{};
    float hitRadius{};
    float angularSpeed{};
    float turnThreshold{};
    std::string solverType;   // "analytical" | "table"
};

struct SimStep {
    Coord pos{};
    float direction{};
    int   state{};
    int   targetIdx{};
    Coord dropPoint{};
    Coord aimPoint{};
    Coord predictedTarget{};
    float timeSecSinceStart{};
};

// Лише поточна позиція та швидкість — без масиву траєкторії.
struct Target {
    Coord pos{};
    Coord velocity{};
};
