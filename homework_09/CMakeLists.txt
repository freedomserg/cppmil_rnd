cmake_minimum_required(VERSION 3.20)
project(drone_sim_v3 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(drone_sim_core_v3
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/providers/JsonTargetProvider.cpp
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
target_include_directories(drone_sim_core_v3 PUBLIC include)

add_executable(drone_sim_v3 src/main.cpp)

target_link_libraries(drone_sim_v3 PRIVATE drone_sim_core_v3)

target_compile_options(drone_sim_v3 PRIVATE -Wall -Wextra -pedantic)
