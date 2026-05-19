#!/usr/bin/env python3

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node


class WaitForSimController(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_sim_controller")
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "rx1_joint_trajectory_controller/follow_joint_trajectory",
        )

    def wait(self) -> None:
        self.get_logger().info(
            "Waiting for rx1_joint_trajectory_controller action server before starting MoveIt"
        )
        attempts = 0
        while rclpy.ok():
            if self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(
                    "Trajectory action server is available; starting MoveIt side"
                )
                return

            attempts += 1
            if attempts % 5 == 0:
                self.get_logger().info(
                    "Still waiting for rx1_joint_trajectory_controller/follow_joint_trajectory"
                )


def main() -> None:
    rclpy.init()
    node = WaitForSimController()
    try:
        node.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
