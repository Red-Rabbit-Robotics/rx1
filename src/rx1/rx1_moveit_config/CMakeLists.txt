cmake_minimum_required(VERSION 3.8)
project(rx1_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_joint_trajectory_controller.py
  scripts/joint_state_timestamp_republisher.py
  scripts/wait_for_sim_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
