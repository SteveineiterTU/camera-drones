cmake_minimum_required(VERSION 2.8.3)
project(octomap_path_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

# ompl
find_package(ompl REQUIRED)
#   checking ompl cmake variables while running: "catkin build"
# message(WARNING "OMPL_INCLUDE_DIRS: " ${OMPL_INCLUDE_DIRS})
# message(WARNING "OMPL_LIBRARY_DIR: " ${OMPL_LIBRARY_DIR})
# message(WARNING "OMPL_LIBRARIES: " ${OMPL_LIBRARIES})
include_directories( ${OMPL_INCLUDE_DIRS} )
link_directories( ${OMPL_LIBRARY_DIR} )
set(EXTRA_LIBS
  ${EXTRA_LIBS}
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

SET( SRC
  src/${PROJECT_NAME}/${PROJECT_NAME}_ros.cpp
)

SET( HDR
  include/${PROJECT_NAME}/${PROJECT_NAME}_ros.h
  include/${PROJECT_NAME}/octomap_helper_functions.h
)

cs_add_library(${PROJECT_NAME}_ros SHARED ${SRC} ${HDR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_ros
    ${EXTRA_LIBS}
)

# -----------------------------
# --- dynamic edt 3D
# set(dynamicEDT3D_DIR ../dynamic_EDT3D)
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros
    ${DYNAMICEDT3D_LIBRARIES}
)
# --- octomap ---
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros
    ${OCTOMAP_LIBRARIES}
)

set (EXTRA_LIBS
    ${EXTRA_LIBS}
    ${PROJECT_NAME}_ros)

cs_add_executable(${PROJECT_NAME}_ros_node src/main_ros_node_octomap.cpp)

target_link_libraries(${PROJECT_NAME}_ros_node ${EXTRA_LIBS})

include_directories(../mav_trajectory_generation/mav_trajectory_generation/include)
include_directories(../mav_trajectory_generation/mav_trajectory_generation_ros/include)

# -----------------------------
##########
# EXPORT #
##########

cs_install()
cs_export()

##
  