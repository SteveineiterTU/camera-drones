cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

cs_add_executable(${PROJECT_NAME}_ros_node src/trajectory_planner/trajectory_planner_ros.cpp src/trajectory_planner/trajectory_planner.cpp)

include_directories(../mav_trajectory_generation/mav_trajectory_generation/include)
include_directories(../mav_trajectory_generation/mav_trajectory_generation_ros/include)
include_directories(../dla3_trajectory_sampler/include)
# -----------------------------
##########
# EXPORT #
##########

cs_install()
cs_export()

##

