#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class JointStateTimestampRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_timestamp_republisher")
        self._latest_clock = None
        self._latest_joint_state = None
        self._logged_clock = False
        self._logged_joint_state = False
        output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            JointState, "joint_states_moveit", output_qos
        )
        self.create_subscription(Clock, "clock", self._handle_clock, qos_profile_sensor_data)
        self.create_subscription(
            JointState,
            "/joint_state_broadcaster/joint_states",
            self._handle_joint_state,
            qos_profile_sensor_data,
        )
        # Some ros2_control setups still publish on the global /joint_states topic
        # even when local topics are requested. Listen there too, but republish onto
        # a dedicated MoveIt-only topic so the zero-stamped raw stream cannot race
        # with the corrected one.
        self.create_subscription(
            JointState,
            "/joint_states",
            self._handle_joint_state,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._publish_latest_joint_state)

    def _handle_clock(self, msg: Clock) -> None:
        self._latest_clock = msg.clock
        if not self._logged_clock:
            self.get_logger().info("Received Gazebo clock; stamping MoveIt joint states")
            self._logged_clock = True

    def _handle_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg
        if not self._logged_joint_state:
            self.get_logger().info(
                f"Received raw joint states with {len(msg.name)} joints; republishing for MoveIt"
            )
            self._logged_joint_state = True

    def _publish_latest_joint_state(self) -> None:
        if self._latest_joint_state is None:
            return

        stamped = JointState()
        stamped.header = self._latest_joint_state.header
        stamped.name = list(self._latest_joint_state.name)
        stamped.position = list(self._latest_joint_state.position)
        stamped.velocity = list(self._latest_joint_state.velocity)
        stamped.effort = list(self._latest_joint_state.effort)

        if self._latest_clock is not None:
            stamped.header.stamp = self._latest_clock
        else:
            stamped.header.stamp = self.get_clock().now().to_msg()

        self._publisher.publish(stamped)


def main() -> None:
    rclpy.init()
    node = JointStateTimestampRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
