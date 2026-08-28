cmake_minimum_required(VERSION 3.20)
project(drone_autopilot CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GPIO-бекенд: за замовчуванням сирий kernel uAPI v2 (без залежностей, чисто
# крос-компілюється); -DUSE_GPIOD=ON перемикає на libgpiod v1 (Pi OS Bookworm).
option(USE_GPIOD "Use libgpiod v1 GPIO backend (Pi OS Bookworm)" OFF)

set(GPIO_BACKEND_SRC src/gpio/RawUapiGpio.cpp)
if(USE_GPIOD)
    set(GPIO_BACKEND_SRC src/gpio/LibgpiodGpio.cpp)
endif()

add_library(drone_autopilot_core
    src/MathUtils.cpp
    src/solvers/BaseBallisticSolver.cpp
    src/solvers/AnalyticalSolver.cpp
    src/solvers/BallisticTable.cpp
    src/solvers/TableSolver.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
    src/states/StateMoving.cpp
    src/MissionPlanner.cpp
    src/uart/UartLink.cpp
    src/uart/UartTargetStore.cpp
    src/control/DroneController.cpp
    src/gpio/GpioFactory.cpp
    ${GPIO_BACKEND_SRC}
)
target_include_directories(drone_autopilot_core PUBLIC include)
# json.hpp + drone_link.h — вендорні, SYSTEM-include щоб clang-tidy їх пропускав.
target_include_directories(drone_autopilot_core SYSTEM PRIVATE third_party)
target_compile_options(drone_autopilot_core PRIVATE -Wall -Wextra -pedantic)

if(USE_GPIOD)
    find_library(GPIOD_LIB gpiod REQUIRED)
    target_link_libraries(drone_autopilot_core PUBLIC ${GPIOD_LIB})
    target_compile_definitions(drone_autopilot_core PUBLIC USE_GPIOD)
endif()

add_executable(drone_autopilot src/main.cpp)
target_link_libraries(drone_autopilot PRIVATE drone_autopilot_core)
target_compile_options(drone_autopilot PRIVATE -Wall -Wextra -pedantic)
