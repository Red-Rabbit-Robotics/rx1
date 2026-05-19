#!/usr/bin/env python3

import tkinter as tk

from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmTorsoHeadController(Node):
    def __init__(self) -> None:
        super().__init__("move_arms_torso_head_with_gui")

        self.right_arm_pub = self.create_publisher(
            JointTrajectory, "/right_arm_position_controller/joint_trajectory", 10
        )
        self.left_arm_pub = self.create_publisher(
            JointTrajectory, "/left_arm_position_controller/joint_trajectory", 10
        )
        self.torso_pub = self.create_publisher(
            JointTrajectory, "/torso_position_controller/joint_trajectory", 10
        )
        self.head_pub = self.create_publisher(
            JointTrajectory, "/head_position_controller/joint_trajectory", 10
        )

        self.right_arm_joints = [
            "right_shoul_base2shoul_joint",
            "right_shoul2shoul_rot_joint",
            "right_arm2armrot_joint",
            "right_armrot2elbow_joint",
            "right_forearm2forearmrot_joint",
            "right_forearmrot2forearm_pitch_joint",
            "right_forearm_pitch2forearm_roll_joint",
        ]
        self.left_arm_joints = [
            "left_shoul_base2shoul_joint",
            "left_shoul2shoul_rot_joint",
            "left_arm2armrot_joint",
            "left_armrot2elbow_joint",
            "left_forearm2forearmrot_joint",
            "left_forearmrot2forearm_pitch_joint",
            "left_forearm_pitch2forearm_roll_joint",
        ]
        self.torso_joints = [
            "base2torso_yaw_joint",
            "torso_yaw2pitch_joint",
            "torso_pitch2roll_joint",
        ]
        self.head_joints = [
            "head_base2neck_yaw_joint",
            "neck_yaw2pitch_joint",
            "neck_pitch2head_depth_cam_mount_joint",
        ]

        self.joint_positions = {
            joint: 0.0
            for joint in self.right_arm_joints
            + self.left_arm_joints
            + self.torso_joints
            + self.head_joints
        }

        self.init_gui()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Joint Angle Controller")

        row = 0
        for joint_name in self.joint_positions:
            label = tk.Label(self.root, text=joint_name)
            label.grid(row=row, column=0)
            slider = tk.Scale(
                self.root,
                from_=-3.14,
                to=3.14,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=300,
                command=lambda val, j=joint_name: self.update_joint_position(j, val),
            )
            slider.grid(row=row, column=1)
            row += 1

        send_button = tk.Button(
            self.root, text="Send Joint Commands", command=self.send_joint_commands
        )
        send_button.grid(row=row, column=0, columnspan=2)

        self.root.after(100, self.spin_ros_once)
        self.root.mainloop()

    def spin_ros_once(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self.root.after(100, self.spin_ros_once)

    def update_joint_position(self, joint_name, value):
        self.joint_positions[joint_name] = float(value)

    def create_trajectory(self, joint_names):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [self.joint_positions[joint] for joint in joint_names]
        point.time_from_start = Duration(seconds=1.0).to_msg()
        traj.points.append(point)
        return traj

    def send_joint_commands(self):
        self.right_arm_pub.publish(self.create_trajectory(self.right_arm_joints))
        self.left_arm_pub.publish(self.create_trajectory(self.left_arm_joints))
        self.torso_pub.publish(self.create_trajectory(self.torso_joints))
        self.head_pub.publish(self.create_trajectory(self.head_joints))
        self.get_logger().info("Joint commands sent based on GUI sliders.")


def main():
    rclpy.init()
    node = ArmTorsoHeadController()
    try:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
