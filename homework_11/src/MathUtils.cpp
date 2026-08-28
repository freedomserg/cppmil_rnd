#include "MathUtils.h"

#include <cmath>
#include <numbers>

auto length(const Coord& c) -> float { return std::hypot(c.x, c.y); }

auto normalize(const Coord& c) -> Coord {
    float len = length(c);
    if (len < 1e-6f) return {0.0f, 0.0f};
    return c / len;
}

auto calcDistance(const Coord& a, const Coord& b) -> float { return length(b - a); }

auto normalizeAngle(float a) -> float {
    constexpr float PI = std::numbers::pi_v<float>;
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

auto calcAccel(float speed, float accelPath) -> float {
    return speed * speed / (2.0f * accelPath);
}

auto accelPathFromSpeed(float v, float a) -> float {
    if (a < 1e-6f) return 0.0f;
    return v * v / (2.0f * a);
}

auto accelTimeFromDist(float d, float a) -> float {
    if (a < 1e-6f) return 0.0f;
    return std::sqrt(2.0f * d / a);
}
