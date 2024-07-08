cmake_minimum_required(VERSION 3.0.2)
project(speed_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  costmap_2d
  speed_filter_msgs
  pallet_truck_msgs
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    costmap_2d
)

# Specify header include paths
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/speed_filter.cpp)