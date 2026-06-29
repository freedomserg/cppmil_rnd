#pragma once

#include "Types.h"

inline constexpr float GRAVITY = 9.81f;

[[nodiscard]] auto length(const Coord& c) -> float;
[[nodiscard]] auto normalize(const Coord& c) -> Coord;
[[nodiscard]] auto calcDistance(const Coord& a, const Coord& b) -> float;

[[nodiscard]] auto normalizeAngle(float a) -> float;
[[nodiscard]] auto calcAccel(float speed, float accelPath) -> float;
[[nodiscard]] auto accelPathFromSpeed(float v, float a) -> float;
[[nodiscard]] auto accelTimeFromDist(float d, float a) -> float;
