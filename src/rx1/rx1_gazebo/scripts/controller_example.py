#!/usr/bin/env python3

from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmTorsoHeadController(Node):
    def __init__(self) -> None:
        super().__init__("move_arms_torso_head_to_zero")

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

        self.timer = self.create_timer(5.0, self.move_all_to_zero)

    def create_trajectory(self, joint_names, pos):
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [pos] * len(joint_names)
        point.time_from_start = Duration(seconds=2.0).to_msg()
        traj.points.append(point)
        return traj

    def move_all_to_zero(self):
        self.right_arm_pub.publish(self.create_trajectory(self.right_arm_joints, 0.0))
        self.left_arm_pub.publish(self.create_trajectory(self.left_arm_joints, 0.0))
        self.torso_pub.publish(self.create_trajectory(self.torso_joints, 0.0))
        self.head_pub.publish(self.create_trajectory(self.head_joints, 0.0))
        self.get_logger().info("Sent zero-position trajectories.")


def main():
    rclpy.init()
    node = ArmTorsoHeadController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
