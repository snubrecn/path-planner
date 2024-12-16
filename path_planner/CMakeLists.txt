cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(path_planner)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB_RECURSE LIB_SRC "*.cc")
file(GLOB_RECURSE LIB_HDR "*.h")

add_library(${PROJECT_NAME} ${LIB_SRC} ${LIB_HDR})