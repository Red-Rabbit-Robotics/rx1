################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(rx1_ik)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  std_msgs
  tf2  
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  tf2_kdl
  sensor_msgs
  pluginlib
  ik_solver_lib
  kdl_parser
  geometry_msgs  
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS roscpp geometry_msgs interactive_markers tf2_msgs tf2_ros sensor_msgs pluginlib ik_solver_lib kdl_parser
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/rx1_ik.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(rx1_ik_node src/rx1_ik_node.cpp)
add_dependencies(rx1_ik_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rx1_ik_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
