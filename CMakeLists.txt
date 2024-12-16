cmake_minimum_required(VERSION 3.10)

project(path_planner_binary)

add_subdirectory(path_planner)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_executable(${PROJECT_NAME} main.cc)
target_link_libraries(${PROJECT_NAME} path_planner ${OpenCV_LIBS})
