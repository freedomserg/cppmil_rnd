cmake_minimum_required(VERSION 3.20)
project(drone_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(drone_sim main.cpp)

# json.hpp — вендорна single-header бібліотека. Позначаємо її як SYSTEM,
# щоб clang-tidy її не аналізував (системні хедери пропускаються). Включається
# як <json.hpp>.
target_include_directories(drone_sim SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_options(drone_sim PRIVATE -Wall -Wextra -pedantic)
