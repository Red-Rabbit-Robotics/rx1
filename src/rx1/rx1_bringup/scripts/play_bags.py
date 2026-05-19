#!/usr/bin/env python3

import subprocess


ROSBAGS = [
    "/home/lingkang/rosbag/dual_arm_hand_bag.bag",
    "/home/lingkang/rosbag/command_head.bag",
    "/home/lingkang/rosbag/right_arm_2.bag",
    "/home/lingkang/rosbag/command_head.bag",
    "/home/lingkang/rosbag/dual_arm_hand_bag.bag",
    "/home/lingkang/rosbag/command_head.bag",
    "/home/lingkang/rosbag/dual_arm_hand_bag.bag",
    "/home/lingkang/rosbag/command_head.bag",
    "/home/lingkang/rosbag/dual_arm_hand_bag.bag",
    "/home/lingkang/rosbag/command_head.bag",
]


def play_rosbag(bag_file: str) -> None:
    print(f"Playing {bag_file}")
    process = subprocess.Popen(["ros2", "bag", "play", "--clock", bag_file])
    process.wait()


if __name__ == "__main__":
    for _ in range(5):
        for bag in ROSBAGS:
            play_rosbag(bag)
