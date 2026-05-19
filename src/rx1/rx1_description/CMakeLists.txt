cmake_minimum_required(VERSION 3.8)
project(rx1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
