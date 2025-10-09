cmake_minimum_required(VERSION 3.0.2)
project(mm_tf2_broadcaster)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  mm_common
  roscpp
  tf2
  tf2_ros
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
  LIBRARIES mm_tf2_broadcaster
  CATKIN_DEPENDS roscpp tf2 mm_common tf2_ros nav_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mm_tf2_broadcaster 
  src/mm_tf2_broadcaster/main.cpp
  src/mm_tf2_broadcaster/tf2_broadcaster.cpp
)

add_dependencies( mm_tf2_broadcaster mm_tf2_broadcaster)

target_link_libraries( mm_tf2_broadcaster ${catkin_LIBRARIES}
)

add_executable(mm_tf2_static_broadcaster 
  src/mm_tf2_static_broadcaster/main.cpp
  src/mm_tf2_static_broadcaster/tf2_static_broadcaster.cpp
)

add_dependencies( mm_tf2_static_broadcaster mm_tf2_static_broadcaster)

target_link_libraries( mm_tf2_static_broadcaster ${catkin_LIBRARIES}
)
