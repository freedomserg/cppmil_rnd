#pragma once

#include "gpio/IGpio.h"

#include <memory>
#include <string>

// Обирає GPIO-бекенд за CMake-опцією USE_GPIOD (libgpiod v1 / сирий uAPI).
auto makeGpio(const std::string& chipName, unsigned startLine, unsigned dropLine)
    -> std::unique_ptr<IGpio>;
