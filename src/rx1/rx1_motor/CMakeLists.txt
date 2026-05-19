cmake_minimum_required(VERSION 3.8)
project(rx1_motor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(feetech_lib REQUIRED)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/rx1_motor.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  feetech_lib
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(rx1_motor_node src/rx1_motor_node.cpp)
target_compile_features(rx1_motor_node PUBLIC cxx_std_17)
target_link_libraries(rx1_motor_node ${PROJECT_NAME})
ament_target_dependencies(rx1_motor_node
  rclcpp
)

install(TARGETS ${PROJECT_NAME} rx1_motor_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  feetech_lib
  rclcpp
  sensor_msgs
  std_msgs
)
ament_package()
