cmake_minimum_required(VERSION 3.20)
project(drone_sim_v4 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(drone_sim_core_v4
    src/MathUtils.cpp
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/providers/ThreadSafeTargetProvider.cpp
    src/physics/DronePhysics.cpp
    src/solvers/BaseBallisticSolver.cpp
    src/solvers/AnalyticalSolver.cpp
    src/solvers/BallisticTable.cpp
    src/solvers/TableSolver.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
    src/states/StateMoving.cpp
    src/MissionProcessor.cpp
)
target_include_directories(drone_sim_core_v4 PUBLIC include)
# json.hpp — вендорна бібліотека, SYSTEM-include щоб clang-tidy її пропускав.
target_include_directories(drone_sim_core_v4 SYSTEM PRIVATE third_party)
target_link_libraries(drone_sim_core_v4 PUBLIC Threads::Threads)
target_compile_options(drone_sim_core_v4 PRIVATE -Wall -Wextra -pedantic)

add_executable(drone_sim_v4 src/main.cpp)
target_link_libraries(drone_sim_v4 PRIVATE drone_sim_core_v4)
target_compile_options(drone_sim_v4 PRIVATE -Wall -Wextra -pedantic)

# Опційний ThreadSanitizer для виявлення гонок: cmake ... -DDRONE_SIM_TSAN=ON
option(DRONE_SIM_TSAN "Build homework_10 with ThreadSanitizer" OFF)
if(DRONE_SIM_TSAN)
    target_compile_options(drone_sim_core_v4 PUBLIC -fsanitize=thread -g)
    target_link_options(drone_sim_core_v4 PUBLIC -fsanitize=thread)
endif()
