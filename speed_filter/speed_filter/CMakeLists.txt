cmake_minimum_required(VERSION 3.0.2)
project(speed_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  costmap_2d
  speed_filter_msgs
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

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})