################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_Minimum_required(VERSION 2.8.3)
project(rx1_motor_lib)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  feetech_lib
  roscpp
  rospy
  std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rx1_motor_lib
 CATKIN_DEPENDS feetech_lib roscpp rospy std_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${PROJECT_SOURCE_DIR}/include/rx1_motor_lib
  ${catkin_INCLUDE_DIRS}
)

file(GLOB hdrs include/rx1_motor_lib/*.hpp)
file(GLOB srs src/*.cpp)

add_library(rx1_motor_lib ${hdrs} ${srs})
add_dependencies(rx1_motor_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rx1_motor_lib
  ${catkin_LIBRARIES}
)
