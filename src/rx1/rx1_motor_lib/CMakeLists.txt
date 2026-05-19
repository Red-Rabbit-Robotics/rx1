cmake_minimum_required(VERSION 3.8)
project(rx1_motor_lib)

find_package(ament_cmake REQUIRED)
find_package(feetech_lib REQUIRED)

add_library(rx1_motor_lib src/rx_motor_lib.cpp)
target_compile_features(rx1_motor_lib PUBLIC cxx_std_17)
target_include_directories(rx1_motor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rx1_motor_lib feetech_lib)

install(TARGETS rx1_motor_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(feetech_lib)
ament_package()
