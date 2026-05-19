#!/usr/bin/env python3

import time
from typing import Dict, List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeJointTrajectoryController(Node):
    def __init__(self) -> None:
        super().__init__("rx1_fake_controller")
        self._joint_names = [
            "base2torso_yaw_joint",
            "torso_yaw2pitch_joint",
            "torso_pitch2roll_joint",
            "right_shoul_base2shoul_joint",
            "right_shoul2shoul_rot_joint",
            "right_arm2armrot_joint",
            "right_armrot2elbow_joint",
            "right_forearm2forearmrot_joint",
            "right_forearmrot2forearm_pitch_joint",
            "right_forearm_pitch2forearm_roll_joint",
            "left_shoul_base2shoul_joint",
            "left_shoul2shoul_rot_joint",
            "left_arm2armrot_joint",
            "left_armrot2elbow_joint",
            "left_forearm2forearmrot_joint",
            "left_forearmrot2forearm_pitch_joint",
            "left_forearm_pitch2forearm_roll_joint",
            "head_base2neck_yaw_joint",
            "neck_yaw2pitch_joint",
            "neck_pitch2head_depth_cam_mount_joint",
            "head_depth_cam_mount2right_ear_joint",
            "head_depth_cam_mount2left_ear_joint",
        ]
        self._positions: Dict[str, float] = {joint: 0.0 for joint in self._joint_names}
        self._velocities: Dict[str, float] = {joint: 0.0 for joint in self._joint_names}
        self._active_goal = None

        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._publish_timer = self.create_timer(0.05, self._publish_joint_states)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "rx1_fake_controller/follow_joint_trajectory",
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
        )

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = [self._positions[joint] for joint in self._joint_names]
        msg.velocity = [self._velocities[joint] for joint in self._joint_names]
        self._joint_state_pub.publish(msg)

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        if self._active_goal is not None:
            self.get_logger().warn("Rejecting trajectory goal because another goal is active.")
            return GoalResponse.REJECT

        unknown_joints = [joint for joint in goal_request.trajectory.joint_names if joint not in self._positions]
        if unknown_joints:
            self.get_logger().warn(f"Rejecting trajectory with unknown joints: {unknown_joints}")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        self._active_goal = goal_handle
        trajectory = goal_handle.request.trajectory
        joint_names: List[str] = list(trajectory.joint_names)
        result = FollowJointTrajectory.Result()

        if not trajectory.points:
            goal_handle.succeed()
            self._active_goal = None
            return result

        start_time = self.get_clock().now()
        previous_time_sec = 0.0

        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._active_goal = None
                return result

            target_time_sec = (
                float(point.time_from_start.sec) +
                float(point.time_from_start.nanosec) / 1e9
            )
            sleep_time = max(0.0, target_time_sec - previous_time_sec)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            previous_time_sec = target_time_sec

            for index, joint in enumerate(joint_names):
                self._positions[joint] = point.positions[index]
                if index < len(point.velocities):
                    self._velocities[joint] = point.velocities[index]
                else:
                    self._velocities[joint] = 0.0

            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = (start_time + Duration(seconds=target_time_sec)).to_msg()
            feedback.joint_names = joint_names
            feedback.actual.positions = [self._positions[joint] for joint in joint_names]
            feedback.actual.velocities = [self._velocities[joint] for joint in joint_names]
            feedback.desired = point
            feedback.error.positions = [0.0] * len(joint_names)
            feedback.error.velocities = [0.0] * len(joint_names)
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        for joint in joint_names:
            self._velocities[joint] = 0.0
        self._active_goal = None
        return result


def main() -> None:
    rclpy.init()
    node = FakeJointTrajectoryController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
