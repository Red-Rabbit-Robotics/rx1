cmake_minimum_required(VERSION 3.8)
project(feetech_lib)

find_package(ament_cmake REQUIRED)

file(GLOB hdrs include/feetech_lib/*.h)
file(GLOB srs src/*.cpp)

add_library(feetech_lib ${hdrs} ${srs})
target_compile_features(feetech_lib PUBLIC cxx_std_17)
target_include_directories(feetech_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/feetech_lib>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS feetech_lib
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
ament_package()
