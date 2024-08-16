# rx1

## Introduction
rx1 is a ROS1 meta package containing basic functioning packages of the RX1 humanoid including the following:  

* rx1_description: URDF file  
* rx1_motor: Feetech servo motor control
* rx1_ik: Inverse Kinematics demo
* rx1_bringup: Launch everything

## Installation
Assuming you have already had ROS 1 Noetic or Melodic installed in your Ubuntu computer.

Dependencies:

* [feetech_lib](https://github.com/Red-Rabbit-Robotics/feetech_lib)
* [ik\_solver\_lib](https://github.com/Red-Rabbit-Robotics/ik_solver_lib)
* other packages listed in the package.xml files you can install through:  
`rosdep install --from-paths src --ignore-src -r -y`

## Demo
1. Test the URDF  
`roslaunch rx1_description urdf_test.launch`  
You should be able control the joint angles through sliders.  
![image](https://github.com/Red-Rabbit-Robotics/rx1/blob/master/media/urdf_test.gif)  

2. Test the inverse kinematics  
`roslaunch rx1_description urdf.launch`  
`roslaunch rx1_ik rx1_ik_marker.launch`  
You should be able click and drag the end effector poses to play with the ik.  
![image](https://github.com/Red-Rabbit-Robotics/rx1/blob/master/media/ik.gif)  

3. Test with the actual robots  
Go to rx1_motor.launch file and modify the USB port value to the corresponding one of your Feetech servo controller board.  
Then, after doing either 1 or 2 above, type:  
`roslaunch rx1_motor rx1_motor.launch`   
You should be able to see the actual robot move based on your commands.

## Extra notes

1. Note that some of the mesh files don't match exactly the latest CAD file of the RX1 Humanoid robot due to development overtime. The overall shape stays the same though.
2. There is no Gazebo Simulation at the moment. And the inertia values in the urdf are placeholders.
3. To launch everything by `roslaunch rx1_bringup bringup.launch` , you need to have the depth camera's ROS package [OrbbecSDK_ROS1](https://github.com/orbbec/OrbbecSDK_ROS1) installed as well.