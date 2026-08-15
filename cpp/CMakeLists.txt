cmake_minimum_required(VERSION 3.15)
project(smart_route_eta_engine CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(ENGINE_SOURCES
    src/graph.cpp
    src/weight_strategy.cpp
    src/pathfinder.cpp
    src/snap.cpp
    src/yen_ksp.cpp
    src/route_validator.cpp
)

add_executable(route_engine src/main.cpp ${ENGINE_SOURCES})
target_include_directories(route_engine PRIVATE include)

add_executable(benchmark src/benchmark.cpp ${ENGINE_SOURCES})
target_include_directories(benchmark PRIVATE include)

add_executable(test_validation_rules tests/test_validation_rules.cpp src/route_validator.cpp src/graph.cpp)
target_include_directories(test_validation_rules PRIVATE include)
