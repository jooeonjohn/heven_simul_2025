cmake_minimum_required(VERSION 3.0.2)
project(mm_pose_parser)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mm_common
  ublox_msgs
  sensor_msgs
  geometry_msgs
  morai_msgs
  tf
  geodesy
)

find_package(PkgConfig REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES mm_pose_parser
 CATKIN_DEPENDS roscpp std_msgs mm_common  ublox_msgs sensor_msgs geometry_msgs morai_msgs tf geodesy
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mm_pose_parser
src/mm_pose_parser/main.cpp
src/mm_pose_parser/pose_parser.cpp
)

add_dependencies(mm_pose_parser mm_pose_parser
)

target_link_libraries(mm_pose_parser
  ${catkin_LIBRARIES}
)
