#include "rx1_ik/rx1_ik.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include <cmath>
#include <functional>
#include <vector>

using std::placeholders::_1;

Rx1Ik::Rx1Ik()
: rclcpp::Node("rx1_ik"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, this, false),
  tf_br_(this)
{
  marker_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    "end_effector_marker", this);

  const double ik_timeout = this->declare_parameter<double>("ik_timeout", 0.005);
  const double eps = this->declare_parameter<double>("eps", 1e-3);
  chain_start_ = this->declare_parameter<std::string>("chain_start", "head_base_link");
  chain_r_end_ = this->declare_parameter<std::string>("chain_r_end", "right_hand_link");
  chain_l_end_ = this->declare_parameter<std::string>("chain_l_end", "left_hand_link");
  urdf_param_ = this->declare_parameter<std::string>("urdf_param", "robot_description");
  max_angle_change_ = this->declare_parameter<double>("max_angle_change", 0.3);
  tracking_timeout_ = this->declare_parameter<double>("tracking_timeout", 1.0);

  try {
    ik_loader_r_ptr_ = std::make_unique<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>>(
      "ik_solver_lib", "ik_solver_plugin::IKSolverBase");
    ik_solver_r_ptr_ = ik_loader_r_ptr_->createSharedInstance("ik_solver_plugin::TracIKSolver");
    ik_solver_r_ptr_->initialize(chain_start_, chain_r_end_, urdf_param_, ik_timeout, eps);
    right_last_ik_time_ = this->now().seconds() - tracking_timeout_;

    ik_loader_l_ptr_ = std::make_unique<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>>(
      "ik_solver_lib", "ik_solver_plugin::IKSolverBase");
    ik_solver_l_ptr_ = ik_loader_l_ptr_->createSharedInstance("ik_solver_plugin::TracIKSolver");
    ik_solver_l_ptr_->initialize(chain_start_, chain_l_end_, urdf_param_, ik_timeout, eps);
    left_last_ik_time_ = this->now().seconds() - tracking_timeout_;

    RCLCPP_INFO(get_logger(), "TracIKSolver plugin loaded and initialized successfully.");
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to create the TracIKSolver plugin. Error: %s", ex.what());
    throw;
  }

  left_joint_state_pub_ = this->create_publisher<JointState>("left_arm_joint_states", 10);
  right_joint_state_pub_ = this->create_publisher<JointState>("right_arm_joint_states", 10);

  right_gripper_pose_sub_ = this->create_subscription<Pose>(
    "right_gripper_pose", 10, std::bind(&Rx1Ik::rightGripperPoseCallback, this, _1));
  left_gripper_pose_sub_ = this->create_subscription<Pose>(
    "left_gripper_pose", 10, std::bind(&Rx1Ik::leftGripperPoseCallback, this, _1));

  JointState right_joint_state_msg;
  right_joint_state_msg.header.stamp = this->now();
  right_joint_state_msg.name = {
    "right_shoul_base2shoul_joint",
    "right_shoul2shoul_rot_joint",
    "right_arm2armrot_joint",
    "right_armrot2elbow_joint",
    "right_forearm2forearmrot_joint",
    "right_forearmrot2forearm_pitch_joint",
    "right_forearm_pitch2forearm_roll_joint"};
  right_joint_state_msg.position = {0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0};
  right_joint_state_pub_->publish(right_joint_state_msg);
  right_prev_joint_state_msg_ = right_joint_state_msg;
  right_cur_joint_state_msg_ = right_joint_state_msg;

  JointState left_joint_state_msg;
  left_joint_state_msg.header.stamp = this->now();
  left_joint_state_msg.name = {
    "left_shoul_base2shoul_joint",
    "left_shoul2shoul_rot_joint",
    "left_arm2armrot_joint",
    "left_armrot2elbow_joint",
    "left_forearm2forearmrot_joint",
    "left_forearmrot2forearm_pitch_joint",
    "left_forearm_pitch2forearm_roll_joint"};
  left_joint_state_msg.position = {0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0};
  left_joint_state_pub_->publish(left_joint_state_msg);
  left_prev_joint_state_msg_ = left_joint_state_msg;
  left_cur_joint_state_msg_ = left_joint_state_msg;

  world_to_base_tf_stamped_.header.stamp = this->now();
  world_to_base_tf_stamped_.header.frame_id = "map";
  world_to_base_tf_stamped_.child_frame_id = "base_link";
  world_to_base_tf_stamped_.transform.rotation.w = 1.0;
  tf_br_.sendTransform(world_to_base_tf_stamped_);

  initializeInteractiveMarker();
}

void Rx1Ik::make6DofMarker(visualization_msgs::msg::InteractiveMarker & int_marker)
{
  int_marker.scale = 0.15;
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1.0;
  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.y = 0.0;
  control.orientation.z = 1.0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.x = 1.0;
  control.orientation.z = 0.0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.y = 0.0;
  control.orientation.z = 1.0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
}

void Rx1Ik::initializeInteractiveMarker()
{
  int_marker_r_.header.frame_id = chain_start_;
  int_marker_r_.name = "right_end_effector";
  int_marker_r_.description = "Right End Effector Control";

  int_marker_l_.header.frame_id = chain_start_;
  int_marker_l_.name = "left_end_effector";
  int_marker_l_.description = "Left End Effector Control";

  make6DofMarker(int_marker_r_);
  make6DofMarker(int_marker_l_);

  marker_server_->insert(int_marker_r_, std::bind(&Rx1Ik::markerRightCallback, this, _1));
  marker_server_->insert(int_marker_l_, std::bind(&Rx1Ik::markerLeftCallback, this, _1));
  marker_server_->applyChanges();
}

void Rx1Ik::markerRightCallback(const InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
  if (feedback->event_type != InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }

  KDL::Frame desired_pose;
  tf2::fromMsg(feedback->pose, desired_pose);

  KDL::JntArray result_joint_positions;
  if (!ik_solver_r_ptr_->solveIK(desired_pose, result_joint_positions)) {
    RCLCPP_WARN(get_logger(), "Failed to find right arm IK solution");
    return;
  }

  bool success = true;
  for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
    if (std::abs(right_prev_joint_state_msg_.position[i] - result_joint_positions(i)) > max_angle_change_) {
      success = false;
    }
  }

  if (success) {
    for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
      right_prev_joint_state_msg_.position[i] =
        right_prev_joint_state_msg_.position[i] * 0.9 + result_joint_positions(i) * 0.1;
    }
    RCLCPP_INFO(get_logger(), "Succeed finding right arm IK solution");
  } else {
    RCLCPP_INFO(get_logger(), "Succeed finding right arm IK solution but ditch the result to reduce shake");
  }
}

void Rx1Ik::markerLeftCallback(const InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
  if (feedback->event_type != InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }

  KDL::Frame desired_pose;
  tf2::fromMsg(feedback->pose, desired_pose);

  KDL::JntArray result_joint_positions;
  if (!ik_solver_l_ptr_->solveIK(desired_pose, result_joint_positions)) {
    RCLCPP_WARN(get_logger(), "Failed to find left arm IK solution");
    return;
  }

  bool success = true;
  for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
    if (std::abs(left_prev_joint_state_msg_.position[i] - result_joint_positions(i)) > max_angle_change_) {
      success = false;
    }
  }

  if (success) {
    for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
      left_prev_joint_state_msg_.position[i] =
        left_prev_joint_state_msg_.position[i] * 0.9 + result_joint_positions(i) * 0.1;
    }
    RCLCPP_INFO(get_logger(), "Succeed finding left arm IK solution");
  } else {
    RCLCPP_INFO(get_logger(), "Succeed finding left arm IK solution but ditch the result to reduce shake");
  }
}

void Rx1Ik::rightGripperPoseCallback(const Pose::SharedPtr msg)
{
  KDL::Frame desired_pose;
  tf2::fromMsg(*msg, desired_pose);

  KDL::JntArray result_joint_positions;
  const double before_ik_time = this->now().seconds();
  const bool ik_solved = ik_solver_r_ptr_->solveIK(desired_pose, result_joint_positions);
  RCLCPP_INFO(get_logger(), "right ik took %f sec", this->now().seconds() - before_ik_time);

  if (!ik_solved) {
    RCLCPP_WARN(get_logger(), "Failed to find right IK solution");
    return;
  }

  bool success = true;
  if ((this->now().seconds() - right_last_ik_time_) < tracking_timeout_) {
    for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
      if (std::abs(right_prev_joint_state_msg_.position[i] - result_joint_positions(i)) > max_angle_change_) {
        success = false;
      }
    }
  }

  if (success) {
    for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
      right_prev_joint_state_msg_.position[i] = result_joint_positions(i);
    }
    right_last_ik_time_ = this->now().seconds();
    RCLCPP_INFO(get_logger(), "Succeed finding right IK solution");
  } else {
    RCLCPP_INFO(get_logger(), "Succeed finding right IK solution but ditch the result to reduce shake");
  }
}

void Rx1Ik::leftGripperPoseCallback(const Pose::SharedPtr msg)
{
  KDL::Frame desired_pose;
  tf2::fromMsg(*msg, desired_pose);

  KDL::JntArray result_joint_positions;
  if (!ik_solver_l_ptr_->solveIK(desired_pose, result_joint_positions)) {
    RCLCPP_WARN(get_logger(), "Failed to find left IK solution");
    return;
  }

  bool success = true;
  if ((this->now().seconds() - left_last_ik_time_) < tracking_timeout_) {
    for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
      if (std::abs(left_prev_joint_state_msg_.position[i] - result_joint_positions(i)) > max_angle_change_) {
        success = false;
      }
    }
  }

  if (success) {
    for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
      left_prev_joint_state_msg_.position[i] = result_joint_positions(i);
    }
    left_last_ik_time_ = this->now().seconds();
    RCLCPP_INFO(get_logger(), "Succeed finding left IK solution");
  } else {
    RCLCPP_INFO(get_logger(), "Succeed finding left IK solution but ditch the result to reduce shake");
  }
}

void Rx1Ik::update()
{
  for (size_t i = 0; i < right_cur_joint_state_msg_.position.size(); ++i) {
    right_cur_joint_state_msg_.position[i] =
      right_cur_joint_state_msg_.position[i] * 0.9 + right_prev_joint_state_msg_.position[i] * 0.1;
  }
  right_cur_joint_state_msg_.header.stamp = this->now();
  right_joint_state_pub_->publish(right_cur_joint_state_msg_);

  for (size_t i = 0; i < left_cur_joint_state_msg_.position.size(); ++i) {
    left_cur_joint_state_msg_.position[i] =
      left_cur_joint_state_msg_.position[i] * 0.9 + left_prev_joint_state_msg_.position[i] * 0.1;
  }
  left_cur_joint_state_msg_.header.stamp = this->now();
  left_joint_state_pub_->publish(left_cur_joint_state_msg_);

  world_to_base_tf_stamped_.header.stamp = this->now();
  tf_br_.sendTransform(world_to_base_tf_stamped_);

  PoseStamped pose;
  if (getLinkPose(chain_start_, chain_r_end_, pose)) {
    int_marker_r_.pose = pose.pose;
    marker_server_->insert(int_marker_r_);
    marker_server_->applyChanges();
  }
  if (getLinkPose(chain_start_, chain_l_end_, pose)) {
    int_marker_l_.pose = pose.pose;
    marker_server_->insert(int_marker_l_);
    marker_server_->applyChanges();
  }
}

void Rx1Ik::spin()
{
  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    update();
    rate.sleep();
  }
}

bool Rx1Ik::getLinkPose(const std::string & frame, const std::string & link, PoseStamped & pose)
{
  try {
    const auto transform_stamped = tf_buffer_.lookupTransform(
      frame, link, rclcpp::Time(0, 0, get_clock()->get_clock_type()), rclcpp::Duration::from_seconds(3.0));
    pose.header.stamp = this->now();
    pose.header.frame_id = frame;
    pose.pose.position.x = transform_stamped.transform.translation.x;
    pose.pose.position.y = transform_stamped.transform.translation.y;
    pose.pose.position.z = transform_stamped.transform.translation.z;
    pose.pose.orientation = transform_stamped.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    return false;
  }
  return true;
}

bool Rx1Ik::getPoseInNewFrame(
  const PoseStamped & old_pose, const std::string & new_frame, PoseStamped & new_pose)
{
  try {
    new_pose = tf_buffer_.transform(old_pose, new_frame, rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "getPoseInNewFrame: %s", ex.what());
    return false;
  }
  return true;
}

Rx1Ik::TransformStamped Rx1Ik::poseToTransformStamped(
  const Pose & pose, const std::string & frame_id, const std::string & child_frame_id)
{
  TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation.x = pose.position.x;
  transform_stamped.transform.translation.y = pose.position.y;
  transform_stamped.transform.translation.z = pose.position.z;
  transform_stamped.transform.rotation = pose.orientation;
  return transform_stamped;
}
