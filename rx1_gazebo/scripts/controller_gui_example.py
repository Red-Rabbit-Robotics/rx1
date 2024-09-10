#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk

class ArmTorsoHeadController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_arms_torso_head_with_gui', anonymous=True)

        # Publishers for the right arm, left arm, torso, and head controllers
        self.right_arm_pub = rospy.Publisher('/right_arm_position_controller/command', JointTrajectory, queue_size=10)
        self.left_arm_pub = rospy.Publisher('/left_arm_position_controller/command', JointTrajectory, queue_size=10)
        self.torso_pub = rospy.Publisher('/torso_position_controller/command', JointTrajectory, queue_size=10)
        self.head_pub = rospy.Publisher('/head_position_controller/command', JointTrajectory, queue_size=10)

        # Joint names for the right arm, left arm, torso, and head
        self.right_arm_joints = [
            'right_shoul_base2shoul_joint',
            'right_shoul2shoul_rot_joint',
            'right_arm2armrot_joint',
            'right_armrot2elbow_joint',
            'right_forearm2forearmrot_joint',
            'right_forearmrot2forearm_pitch_joint',
            'right_forearm_pitch2forearm_roll_joint'
        ]

        self.left_arm_joints = [
            'left_shoul_base2shoul_joint',
            'left_shoul2shoul_rot_joint',
            'left_arm2armrot_joint',
            'left_armrot2elbow_joint',
            'left_forearm2forearmrot_joint',
            'left_forearmrot2forearm_pitch_joint',
            'left_forearm_pitch2forearm_roll_joint'
        ]

        self.torso_joints = [
            'base2torso_yaw_joint',
            'torso_yaw2pitch_joint',
            'torso_pitch2roll_joint'
        ]

        self.head_joints = [
            'head_base2neck_yaw_joint',
            'neck_yaw2pitch_joint',
            'neck_pitch2head_depth_cam_mount_joint'
        ]

        # Variables to hold joint positions from sliders
        self.joint_positions = {
            joint: 0.0 for joint in self.right_arm_joints + self.left_arm_joints + self.torso_joints + self.head_joints
        }

        # Initialize the Tkinter GUI
        self.init_gui()

    def init_gui(self):
        """Initialize the Tkinter GUI with sliders for each joint."""
        self.root = tk.Tk()
        self.root.title("Joint Angle Controller")

        row = 0
        # Create sliders for all joints
        for joint_name in self.joint_positions:
            label = tk.Label(self.root, text=joint_name)
            label.grid(row=row, column=0)
            slider = tk.Scale(self.root, from_=-3.14, to=3.14, resolution=0.01, orient=tk.HORIZONTAL, length=300,
                              command=lambda val, j=joint_name: self.update_joint_position(j, val))
            slider.grid(row=row, column=1)
            row += 1

        # Create a button to send the commands
        send_button = tk.Button(self.root, text="Send Joint Commands", command=self.send_joint_commands)
        send_button.grid(row=row, column=0, columnspan=2)

        # Start the Tkinter main loop
        self.root.mainloop()

    def update_joint_position(self, joint_name, value):
        """Update the joint position when the slider is moved."""
        self.joint_positions[joint_name] = float(value)

    def create_trajectory(self, joint_names):
        """Creates a JointTrajectory message based on the slider values."""
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [self.joint_positions[joint] for joint in joint_names]
        point.time_from_start = rospy.Duration(1.0)  # Set the movement duration

        traj.points.append(point)
        return traj

    def send_joint_commands(self):
        """Send the joint commands based on the slider positions."""
        # Publish the trajectory for the right arm
        right_arm_traj = self.create_trajectory(self.right_arm_joints)
        self.right_arm_pub.publish(right_arm_traj)

        # Publish the trajectory for the left arm
        left_arm_traj = self.create_trajectory(self.left_arm_joints)
        self.left_arm_pub.publish(left_arm_traj)

        # Publish the trajectory for the torso
        torso_traj = self.create_trajectory(self.torso_joints)
        self.torso_pub.publish(torso_traj)

        # Publish the trajectory for the head
        head_traj = self.create_trajectory(self.head_joints)
        self.head_pub.publish(head_traj)

        rospy.loginfo("Joint commands sent based on GUI sliders.")

if __name__ == '__main__':
    try:
        # Initialize the controller
        controller = ArmTorsoHeadController()
    except rospy.ROSInterruptException:
        pass

