cmake_minimum_required(VERSION 3.8)
project(rx1_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller_example.py
  scripts/controller_gui_example.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
