cmake_minimum_required(VERSION 3.20)
project(drone_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(drone_sim main.cpp)

target_compile_options(drone_sim PRIVATE -Wall -Wextra -pedantic)