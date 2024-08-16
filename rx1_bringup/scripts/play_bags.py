#!/usr/bin/env python

import subprocess
import time

# List of ROS bag files to play in sequence
rosbags = [
    "/home/lingkang/rosbag/right_arm_2.bag",
    "/home/lingkang/rosbag/command_head.bag",
    "/home/lingkang/rosbag/dual_arm_1.bag",
    "/home/lingkang/rosbag/command_torso.bag",
    "/home/lingkang/rosbag/right_arm_2.bag",
    "/home/lingkang/rosbag/command_head.bag",
    "/home/lingkang/rosbag/dual_arm_2.bag",
    "/home/lingkang/rosbag/right_arm_2.bag",
    "/home/lingkang/rosbag/command_head.bag",
    "/home/lingkang/rosbag/right_arm_1.bag",
    "/home/lingkang/rosbag/dual_arm_3.bag"
]

# Function to play a single ROS bag file
def play_rosbag(bag_file):
    print(f"Playing {bag_file}")
    process = subprocess.Popen(["rosbag", "play", "--clock", bag_file])
    process.wait()  # Wait for the current bag file to finish playing

if __name__ == '__main__':
    # Set use_sim_time to true
    subprocess.call(["rosparam", "set", "/use_sim_time", "true"])

    # Ensure roscore is running
    try:
        #roscore_process = subprocess.Popen(["roscore"])
        #time.sleep(5)  # Give roscore some time to start

        # Play each ROS bag file in sequence
        for bag in rosbags:
            play_rosbag(bag)
        for bag in rosbags:
            play_rosbag(bag)
        for bag in rosbags:
            play_rosbag(bag)
        for bag in rosbags:
            play_rosbag(bag)
        for bag in rosbags:
            play_rosbag(bag)

    finally:
        # Terminate roscore
        roscore_process.terminate()

