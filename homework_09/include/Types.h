#pragma once

#include <iostream>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

struct Coord {
    float x{};
    float y{};

    Coord operator+(const Coord& o) const { return {x + o.x, y + o.y}; }
    Coord operator-(const Coord& o) const { return {x - o.x, y - o.y}; }
    Coord operator*(float s)        const { return {x * s,   y * s};   }

    Coord operator/(float s) const {
        if (s < 1e-6f) {
            std::cerr << "Error: division by zero in Coord operator/" << std::endl;
            return {0.0f, 0.0f};
        }
        return {x / s, y / s};
    }

    bool operator==(const Coord& o) const {
        return std::fabs(x - o.x) < 1e-6f && std::fabs(y - o.y) < 1e-6f;
    }

    float length() const { return std::hypot(x, y); }

    Coord normalize() const {
        float len = length();
        if (len < 1e-6f) {
            std::cerr << "Error: Coord::normalize called on zero-length vector" << std::endl;
            return {0.0f, 0.0f};
        }
        return *this / len;
    }
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
};

// Non-owning view of one target's position track.
// The data is owned by the ITargetProvider that created this Target.
struct Target {
    const Coord* positions;
    int          timeStepsCount;
};

// ── Physics constant ────────────────────────────────────────────────────────
inline constexpr float GRAVITY = 9.81f;

// ── Shared math utilities ───────────────────────────────────────────────────

// Wraps angle into [-π, π].
inline float normalizeAngle(float a) {
    constexpr float PI = std::numbers::pi_v<float>;
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

// Constant acceleration from rest over accelPath: a = v² / (2·s).
inline float calcAccel(float speed, float accelPath) {
    return speed * speed / (2.0f * accelPath);
}

inline float calcDistance(const Coord& a, const Coord& b) {
    return (b - a).length();
}

// Distance needed to decelerate from v to 0: s = v² / (2·a).
inline float accelPathFromSpeed(float v, float a) {
    if (a < 1e-6f) { std::cerr << "Error: a must be > 0 in accelPathFromSpeed\n"; return 0.0f; }
    return v * v / (2.0f * a);
}

// Time to travel distance d from rest with acceleration a: t = √(2·d / a).
inline float accelTimeFromDist(float d, float a) {
    if (a < 1e-6f) { std::cerr << "Error: a must be > 0 in accelTimeFromDist\n"; return 0.0f; }
    return std::sqrt(2.0f * d / a);
}
