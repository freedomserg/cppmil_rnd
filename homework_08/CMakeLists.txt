cmake_minimum_required(VERSION 3.20)
project(drone_sim_v2 CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(drone_sim_core
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/providers/JsonTargetProvider.cpp
    src/solvers/AnalyticalSolver.cpp
    src/MissionProcessor.cpp
)
target_include_directories(drone_sim_core PUBLIC include)

add_executable(drone_sim_v2 src/main.cpp)

target_link_libraries(drone_sim_v2 PRIVATE drone_sim_core)

target_compile_options(drone_sim_v2 PRIVATE -Wall -Wextra -pedantic)
