#ifndef RX1_IK_H
#define RX1_IK_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <pluginlib/class_loader.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/ros.h>

#include "ik_solver_lib/base/ik_solver_base.h"

class Rx1Ik
{
public:
    Rx1Ik(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    void initializeInteractiveMarker();
    void markerRightCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void markerLeftCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void rightGripperPoseCallback(const geometry_msgs::Pose& msg);
    void leftGripperPoseCallback(const geometry_msgs::Pose& msg);
    void spinOnce();
    void spin();
    void update();

private:
    void make6DofMarker(visualization_msgs::InteractiveMarker &int_marker);
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
 
    interactive_markers::InteractiveMarkerServer marker_server_;
    visualization_msgs::InteractiveMarker int_marker_r_;
    visualization_msgs::InteractiveMarker int_marker_l_;
   
    ros::Subscriber right_gripper_pose_sub_;
    ros::Subscriber left_gripper_pose_sub_;

    ros::Publisher right_joint_state_pub_;
    ros::Publisher left_joint_state_pub_;
    
    std::unique_ptr<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>> ik_loader_r_ptr_;
    boost::shared_ptr<ik_solver_plugin::IKSolverBase> ik_solver_r_ptr_;

    std::unique_ptr<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>> ik_loader_l_ptr_;
    boost::shared_ptr<ik_solver_plugin::IKSolverBase> ik_solver_l_ptr_;

    sensor_msgs::JointState right_prev_joint_state_msg_;
    sensor_msgs::JointState left_prev_joint_state_msg_;

    sensor_msgs::JointState right_cur_joint_state_msg_;
    sensor_msgs::JointState left_cur_joint_state_msg_;
    
    geometry_msgs::TransformStamped world_to_base_tf_stamped_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_br_;

    // Params
    std::string chain_start_, chain_r_end_, chain_l_end_, urdf_param_;
    double max_angle_change_;
    double tracking_timeout_;

    // Ik variables
    double right_last_ik_time_;
    double left_last_ik_time_;

    // util functions
    
    // Get link's pose in frame
    bool getLinkPose(const std::string frame, const std::string link, geometry_msgs::PoseStamped& pose);
    // Turn the pose in frame a to frame b
    bool getPoseInNewFrame(const geometry_msgs::PoseStamped old_pose, const std::string new_frame, geometry_msgs::PoseStamped& new_pose);
    // Turn pose to transform
    geometry_msgs::TransformStamped poseToTransformStamped(const geometry_msgs::Pose& pose, const std::string& frame_id, const std::string& child_frame_id);
};

#endif // RX1_IK_H
