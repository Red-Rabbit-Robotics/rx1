#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmTorsoHeadController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_arms_torso_head_to_zero', anonymous=True)

        # Publishers for the right arm, left arm, torso, and head controllers
        self.right_arm_pub = rospy.Publisher('/right_arm_position_controller/command', JointTrajectory, queue_size=10)
        self.left_arm_pub = rospy.Publisher('/left_arm_position_controller/command', JointTrajectory, queue_size=10)
        self.torso_pub = rospy.Publisher('/torso_position_controller/command', JointTrajectory, queue_size=10)
        self.head_pub = rospy.Publisher('/head_position_controller/command', JointTrajectory, queue_size=10)

        # Define joint names for the right arm
        self.right_arm_joints = [
            'right_shoul_base2shoul_joint',
            'right_shoul2shoul_rot_joint',
            'right_arm2armrot_joint',
            'right_armrot2elbow_joint',
            'right_forearm2forearmrot_joint',
            'right_forearmrot2forearm_pitch_joint',
            'right_forearm_pitch2forearm_roll_joint'
        ]

        # Define joint names for the left arm
        self.left_arm_joints = [
            'left_shoul_base2shoul_joint',
            'left_shoul2shoul_rot_joint',
            'left_arm2armrot_joint',
            'left_armrot2elbow_joint',
            'left_forearm2forearmrot_joint',
            'left_forearmrot2forearm_pitch_joint',
            'left_forearm_pitch2forearm_roll_joint'
        ]

        # Define joint names for the torso
        self.torso_joints = [
            'base2torso_yaw_joint',
            'torso_yaw2pitch_joint',
            'torso_pitch2roll_joint'
        ]

        # Define joint names for the head
        self.head_joints = [
            'head_base2neck_yaw_joint',
            'neck_yaw2pitch_joint',
            'neck_pitch2head_depth_cam_mount_joint'
        ]

        # Set the rate for the loop
        self.rate = rospy.Rate(0.2)  # 0.1 Hz

    def create_trajectory(self, joint_names, pos):
        """Creates a JointTrajectory message with all joints set to 0 position."""
        traj = JointTrajectory()
        traj.joint_names = joint_names

        # Create a trajectory point that sets all joints to position 0
        point = JointTrajectoryPoint()
        point.positions = [pos] * len(joint_names)  # Set all positions to 0
        point.time_from_start = rospy.Duration(2.0)  # Move to zero position in 2 seconds

        traj.points.append(point)
        return traj

    def move_right_arm_to_zero(self):
        """Publishes a trajectory to move the right arm joints to position 0."""
        right_arm_traj = self.create_trajectory(self.right_arm_joints, 0)
        rospy.loginfo("Sending trajectory to move right arm joints to position 0.")
        self.right_arm_pub.publish(right_arm_traj)

    def move_left_arm_to_zero(self):
        """Publishes a trajectory to move the left arm joints to position 0."""
        left_arm_traj = self.create_trajectory(self.left_arm_joints, 0)
        rospy.loginfo("Sending trajectory to move left arm joints to position 0.")
        self.left_arm_pub.publish(left_arm_traj)

    def move_torso_to_zero(self):
        """Publishes a trajectory to move the torso joints to position 0."""
        torso_traj = self.create_trajectory(self.torso_joints, 0)
        rospy.loginfo("Sending trajectory to move torso joints to position 0.")
        self.torso_pub.publish(torso_traj)

    def move_head_to_zero(self):
        """Publishes a trajectory to move the head joints to position 0."""
        head_traj = self.create_trajectory(self.head_joints, 0)
        rospy.loginfo("Sending trajectory to move head joints to position 0.")
        self.head_pub.publish(head_traj)

    def move_all_to_zero(self):
        """Moves both arms, the torso, and the head to zero position."""
        while not rospy.is_shutdown():
            self.move_right_arm_to_zero()
            self.move_left_arm_to_zero()
            self.move_torso_to_zero()
            self.move_head_to_zero()
            self.rate.sleep()  # Sleep to maintain the loop at 10 Hz

if __name__ == '__main__':
    try:
        # Initialize the ArmTorsoHeadController class
        controller = ArmTorsoHeadController()

        # Move all joints (arms, torso, and head) to zero position
        controller.move_all_to_zero()
    except rospy.ROSInterruptException:
        pass

