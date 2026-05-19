# rx1

## Introduction
RX1 Humanoid is an open-source robot project from Red Rabbit Robotics.

This repository is a ROS 2 conversion of the original ROS 1 RX1 project, with a MoveIt 2 setup added for planning and RViz-based manipulation workflows.

References:

* Original project page: [https://red-rabbit-robotics.ghost.io/](https://red-rabbit-robotics.ghost.io/)
* Original ROS 1 repository: [https://github.com/Red-Rabbit-Robotics/rx1](https://github.com/Red-Rabbit-Robotics/rx1)

## Current Status

This repo has been ported toward ROS 2 and currently supports:

* ROS 2 package metadata and `colcon` builds
* RViz robot visualization through `rx1_description`
* MoveIt 2 planning setup through `rx1_moveit_config`
* ROS 2 motor package build through `rx1_motor`
* ROS 2-compatible `feetech_lib` for real servo communication

Current direction:

* MoveIt 2 is the preferred manipulation path
* the old `rx1_ik` package is still present, but MoveIt 2 is the main ROS 2 planning interface

Known gaps:

* Gazebo and MoveIt execution now work on the maintained Jazzy path, but simulator startup still depends on the ROS 2 / Gazebo Harmonic stack being available locally
* `ik_solver_lib` is still needed only if you want to keep the old custom IK workflow
* hardware use still requires the proper serial device, servo IDs, and safe testing on the real robot

## Packages

Main packages in this repository:

* `rx1_description`: URDF/Xacro model, meshes, RViz config, and robot state publishing launches
* `rx1_motor`: ROS 2 motor node for Feetech-based hardware control
* `rx1_ik`: legacy custom IK path kept from the original project
* `rx1_bringup`: top-level launch integration
* `rx1_gazebo`: Gazebo simulation package
* `rx1_moveit_config`: MoveIt 2 configuration, SRDF, RViz config, and demo launch

External dependency used for real servos:

* `feetech_lib`: ROS 2-converted Feetech communication library

## ROS 1 to ROS 2 Notes

Main command and workflow changes:

* `catkin_make` or `catkin build` -> `colcon build`
* `roslaunch <pkg> <file.launch>` -> `ros2 launch <pkg> <file.launch.py>`
* `rosrun <pkg> <node>` -> `ros2 run <pkg> <node>`
* ROS 1 parameter server usage -> ROS 2 declared parameters and YAML parameter files
* `tf` static publishers -> `tf2_ros/static_transform_publisher`
* ROS 1 launch XML -> ROS 2 Python launch files

## Installation

Example workspace setup:

```bash
mkdir -p ~/rx1_ws/src
cd ~/rx1_ws/src
git clone <this-rx1-repo-url> rx1
git clone <feetech-lib-ros2-url-or-local-copy> feetech_lib
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

If you do not need real servo hardware support, `feetech_lib` is not required for MoveIt 2 and basic RViz workflows.

Recommended ROS 2 distro:

* Jazzy on Ubuntu

## Current Launch Defaults

The maintained simulation path now uses optimized collision-model copies by default:

* MoveIt model: `rx1_optimized.urdf.xacro`
* Gazebo model: `rx1_optimized.harmonic.urdf.xacro`

The original model files are still kept in the repository and can be selected manually through launch arguments if needed.

## MoveIt 2 Demo

MoveIt 2 is the recommended way to work with RX1 in ROS 2.

Launch the planning demo:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
ros2 launch rx1_moveit_config demo.launch.py
```

What this gives you:

* RViz with `base_link` as the fixed frame
* `move_group`
* planning groups for `right_arm`, `left_arm`, `dual_arms`, and `head`
* KDL-based kinematics configuration for the supported groups

Notes:

* the orange overlay in RViz is typically the MoveIt goal state visualization, not a collision by itself
* octomap sensor warnings can appear even when planning in RViz is otherwise working
* the optimized model reduces collision-check cost while keeping the original visual meshes

## URDF / RViz Demo

To inspect the robot model without MoveIt:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rx1_description urdf_test.launch.py
```

This starts:

* `robot_state_publisher`
* `joint_state_publisher_gui`
* `rviz2`

## Hardware Servo Control

For real Feetech servo control:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rx1_motor rx1_motor.launch.py
```

Important:

* `feetech_lib` is required for real hardware control
* it is not required for MoveIt 2 planning or basic RViz model visualization
* verify the correct serial port such as `/dev/ttyUSB0` before enabling hardware

## Gazebo

Gazebo support has been ported toward ROS 2 and the maintained Jazzy path now includes:

* default optimized simulation models
* a Gazebo-specific RViz config
* delayed controller startup to avoid `controller_manager` races
* a `gazebo_demo.launch.py` flow that waits for the simulated trajectory action server before starting the MoveIt side

Launch Gazebo only:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
ros2 launch rx1_gazebo rx1_gazebo.launch.py
```

Launch Gazebo with the simulation RViz view:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
ros2 launch rx1_gazebo rx1_gazebo_rviz.launch.py
```

Launch Gazebo with MoveIt planning in RViz:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
ros2 launch rx1_moveit_config gazebo_demo.launch.py
```

What this gives you:

* Gazebo simulation
* controller startup through `gz_ros2_control`
* RViz with MoveIt
* execution through `rx1_joint_trajectory_controller`

Useful launch arguments:

* `robot_x:=... robot_y:=... robot_z:=...` to place the robot in the world
* `moveit_model:=...` and `sim_model:=...` if you want to override the optimized defaults in `gazebo_demo.launch.py`
* `model:=...` if you want to override the optimized default in `rx1_gazebo_rviz.launch.py`

Simulation notes:

* `gazebo_demo.launch.py` now starts the MoveIt side only after the simulated `follow_joint_trajectory` action server is available
* MoveIt uses a stamped `/joint_states_moveit` stream internally to avoid zero-timestamp execution failures from the raw sim state stream
* if RViz camera rendering is unstable on your GPU driver, keep camera displays disabled in MoveIt RViz and use `ros2 run rqt_image_view rqt_image_view` to inspect `/rx1/rgbd_camera/image` or `/rx1/rgbd_camera/depth_image`
* the repeated `base_link` inertia warning from KDL is non-fatal and does not block planning or execution

## Legacy IK Path

The original custom IK path is still present in `rx1_ik`, but MoveIt 2 is the preferred ROS 2 workflow.

Use the legacy IK path only if you specifically want to preserve the original IK behavior and also have a ROS 2-compatible `ik_solver_lib`.

## Extra Notes

1. Some mesh files may not exactly match the latest RX1 CAD state, but the overall robot shape is preserved.
2. `rx1.urdf.xacro` contains placeholder inertia structure in places, while generated URDF content is used for runtime visualization and planning.
3. For ROS 2 use, the practical order is:
   * validate `rx1_description`
   * use `rx1_moveit_config`
   * bring up `rx1_motor` only when moving to real hardware
