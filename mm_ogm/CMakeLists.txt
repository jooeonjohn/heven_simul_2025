cmake_minimum_required(VERSION 3.0.2)
project(mm_ogm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  mm_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ogm
  CATKIN_DEPENDS mm_common nav_msgs roscpp sensor_msgs std_msgs tf2_ros pcl_ros pcl_conversions
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(mm_ogm
  src/main.cpp
  src/mm_ogm.cpp
)

target_link_libraries(mm_ogm
  ${catkin_LIBRARIES}
)

add_dependencies(mm_ogm mm_ogm
)