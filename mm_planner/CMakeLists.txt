cmake_minimum_required(VERSION 3.0.2)
project(mm_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mm_common
  ublox_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  tf
  morai_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES mm_planner
 CATKIN_DEPENDS roscpp std_msgs mm_common ublox_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs tf morai_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)




add_executable(mm_planner
src/mm_planner/main.cpp
src/mm_planner/planner.cpp
src/mm_gps_tracker/gps_tracker.cpp
src/mm_dwa_tracker/mm_dwa_tracker.cpp
)

add_dependencies(mm_planner mm_planner
)

target_link_libraries(mm_planner
  ${catkin_LIBRARIES}
)