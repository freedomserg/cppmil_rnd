#pragma once

#include "Types.h"

#include <utility>

// Окремий модуль керування: перетворює рішення місії {state, angleSpeed} на
// нормовані UART-команди {accel, turnRate} ∈ [-1,1] (чекер множить їх на ліміти).
class DroneController {
public:
    explicit DroneController(DroneConfig cfg) : cfg_(std::move(cfg)) {}

    struct Output {
        float accel;
        float turnRate;
    };

    [[nodiscard]] auto toControl(const DroneCommand& cmd, float speed) const -> Output;

private:
    DroneConfig cfg_;
};
