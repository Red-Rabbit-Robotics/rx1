cmake_minimum_required(VERSION 3.8)
project(rx1_ik)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ik_solver_lib REQUIRED)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/rx1_ik.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  interactive_markers
  ik_solver_lib
  kdl_parser
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_kdl
  tf2_msgs
  tf2_ros
  visualization_msgs
)

add_executable(rx1_ik_node src/rx1_ik_node.cpp)
target_compile_features(rx1_ik_node PUBLIC cxx_std_17)
target_link_libraries(rx1_ik_node ${PROJECT_NAME})
ament_target_dependencies(rx1_ik_node rclcpp)

install(TARGETS ${PROJECT_NAME} rx1_ik_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config include launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  interactive_markers
  ik_solver_lib
  kdl_parser
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_kdl
  tf2_msgs
  tf2_ros
  visualization_msgs
)
ament_package()
