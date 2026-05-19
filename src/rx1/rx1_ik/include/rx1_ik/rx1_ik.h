#ifndef RX1_IK_H
#define RX1_IK_H

#include "ik_solver_lib/base/ik_solver_base.h"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <memory>
#include <string>

class Rx1Ik : public rclcpp::Node
{
public:
  Rx1Ik();

  void initializeInteractiveMarker();
  void spin();
  void update();

private:
  using InteractiveMarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;
  using JointState = sensor_msgs::msg::JointState;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  void markerRightCallback(const InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void markerLeftCallback(const InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void rightGripperPoseCallback(const Pose::SharedPtr msg);
  void leftGripperPoseCallback(const Pose::SharedPtr msg);
  void make6DofMarker(visualization_msgs::msg::InteractiveMarker & int_marker);
  bool getLinkPose(const std::string & frame, const std::string & link, PoseStamped & pose);
  bool getPoseInNewFrame(
    const PoseStamped & old_pose, const std::string & new_frame, PoseStamped & new_pose);
  TransformStamped poseToTransformStamped(
    const Pose & pose, const std::string & frame_id, const std::string & child_frame_id);

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  visualization_msgs::msg::InteractiveMarker int_marker_r_;
  visualization_msgs::msg::InteractiveMarker int_marker_l_;

  rclcpp::Subscription<Pose>::SharedPtr right_gripper_pose_sub_;
  rclcpp::Subscription<Pose>::SharedPtr left_gripper_pose_sub_;
  rclcpp::Publisher<JointState>::SharedPtr right_joint_state_pub_;
  rclcpp::Publisher<JointState>::SharedPtr left_joint_state_pub_;

  std::unique_ptr<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>> ik_loader_r_ptr_;
  std::shared_ptr<ik_solver_plugin::IKSolverBase> ik_solver_r_ptr_;
  std::unique_ptr<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>> ik_loader_l_ptr_;
  std::shared_ptr<ik_solver_plugin::IKSolverBase> ik_solver_l_ptr_;

  JointState right_prev_joint_state_msg_;
  JointState left_prev_joint_state_msg_;
  JointState right_cur_joint_state_msg_;
  JointState left_cur_joint_state_msg_;

  TransformStamped world_to_base_tf_stamped_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_br_;

  std::string chain_start_;
  std::string chain_r_end_;
  std::string chain_l_end_;
  std::string urdf_param_;
  double max_angle_change_;
  double tracking_timeout_;
  double right_last_ik_time_;
  double left_last_ik_time_;
};

#endif  // RX1_IK_H
