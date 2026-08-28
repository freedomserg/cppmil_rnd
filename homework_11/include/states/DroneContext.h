#pragma once

#include "Types.h"

// Знімок телеметрії + входи планування. Стани лише читають і заповнюють
// command; інтеграцію виконує DronePhysics.
struct DroneContext {
    DroneConfig cfg;
    float acceleration = 0.0f;

    Coord pos{};
    float direction = 0.0f;
    float speed     = 0.0f;

    float desiredDir = 0.0f;
    float deltaAngle = 0.0f;

    DroneCommand command{};
};
