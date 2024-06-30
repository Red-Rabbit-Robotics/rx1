#include <iostream>

#include <cmath>

#include "rx1_ik/rx1_ik.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.h>

Rx1Ik::Rx1Ik(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
    : nh_(nh),
      priv_nh_(priv_nh),
      marker_server_("end_effector_marker"),
      tf_listener_(tf_buffer_)
{
    // Load the IK solver plugin
    try
    {
        // Retrieve parameters from the parameter server
        double ik_timeout;
        double eps;

        priv_nh_.param("chain_start", chain_start_, std::string("head_base_link"));
        priv_nh_.param("chain_r_end", chain_r_end_, std::string("right_hand_link"));
        priv_nh_.param("chain_l_end", chain_l_end_, std::string("left_hand_link"));
        priv_nh_.param("urdf_param", urdf_param_, std::string("/robot_description"));
        priv_nh_.param("ik_timeout", ik_timeout, 0.005); // Default timeout
        priv_nh_.param("eps", eps, 1e-3); // Default error
        priv_nh_.param("max_angle_change", max_angle_change_, 0.3);
        priv_nh_.param("tracking_timeout", tracking_timeout_, 1.0);
        
        ROS_INFO("eps: %f", eps);

        // Create IK solver instance
        ik_loader_r_ptr_ = std::make_unique<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>>("ik_solver_lib", "ik_solver_plugin::IKSolverBase");
        ik_solver_r_ptr_ = ik_loader_r_ptr_->createInstance("ik_solver_plugin::TracIKSolver");
        ik_solver_r_ptr_->initialize(chain_start_, chain_r_end_, urdf_param_, ik_timeout, eps);

        right_last_ik_time_ = ros::Time::now().toSec() - tracking_timeout_;
        
        ik_loader_l_ptr_ = std::make_unique<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>>("ik_solver_lib", "ik_solver_plugin::IKSolverBase");
        ik_solver_l_ptr_ = ik_loader_l_ptr_->createInstance("ik_solver_plugin::TracIKSolver");
        ik_solver_l_ptr_->initialize(chain_start_, chain_l_end_, urdf_param_, ik_timeout, eps);
        
        left_last_ik_time_ = ros::Time::now().toSec() - tracking_timeout_;
        
        ROS_INFO("TracIKSolver plugin loaded and initialized successfully.");
    }
    catch (const pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("Failed to create the TracIKSolver plugin. Error: %s", ex.what());
        return;
    }

    // Publisher for joint states
    left_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("right_arm_joint_states", 10);
    right_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("left_arm_joint_states", 10);

    // Initialize subscribers
    right_gripper_pose_sub_ = nh_.subscribe("right_gripper_pose", 10, &Rx1Ik::rightGripperPoseCallback, this);
    left_gripper_pose_sub_ = nh_.subscribe("left_gripper_pose", 10, &Rx1Ik::leftGripperPoseCallback, this);

    // Initialize joint states
    sensor_msgs::JointState right_joint_state_msg;
    right_joint_state_msg.header.stamp = ros::Time::now();
    right_joint_state_msg.name.resize(7);
    right_joint_state_msg.position.resize(7);
    std::vector<std::string> right_joint_name = {"right_shoul_base2shoul_joint",
                                                 "right_shoul2shoul_rot_joint", 
                                                 "right_arm2armrot_joint", 
                                                 "right_armrot2elbow_joint", 
                                                 "right_forearm2forearmrot_joint", 
                                                 "right_forearmrot2forearm_pitch_joint",
                                                 "right_forearm_pitch2forearm_roll_joint"};
    for (int i = 0; i < 7; ++i)
    {
        right_joint_state_msg.position[i] = 0;
        right_joint_state_msg.name[i] = right_joint_name[i];
    }
    right_joint_state_msg.position[3] = -1.57;
    right_joint_state_pub_.publish(right_joint_state_msg);
    right_prev_joint_state_msg_ = right_joint_state_msg;
    

    sensor_msgs::JointState left_joint_state_msg;
    left_joint_state_msg.header.stamp = ros::Time::now();
    left_joint_state_msg.name.resize(7);
    left_joint_state_msg.position.resize(7);
    std::vector<std::string> left_joint_name = {"left_shoul_base2shoul_joint",
                                                "left_shoul2shoul_rot_joint", 
                                                "left_arm2armrot_joint", 
                                                "left_armrot2elbow_joint", 
                                                "left_forearm2forearmrot_joint", 
                                                "left_forearmrot2forearm_pitch_joint",
                                                "left_forearm_pitch2forearm_roll_joint"};
    for (int i = 0; i < 7; ++i)
    {
        left_joint_state_msg.position[i] = 0;
        left_joint_state_msg.name[i] = left_joint_name[i];
    }
    left_joint_state_msg.position[3] = -1.57;
    left_joint_state_pub_.publish(left_joint_state_msg);
    left_prev_joint_state_msg_ = left_joint_state_msg;

    ROS_INFO("Joints initialized");

    // Initialize floating joint
    world_to_base_tf_stamped_.header.stamp = ros::Time::now();
    world_to_base_tf_stamped_.header.frame_id = "map";
    world_to_base_tf_stamped_.child_frame_id = "base_link";
    world_to_base_tf_stamped_.transform.translation.x = 0.0;
    world_to_base_tf_stamped_.transform.translation.y = 0.0;
    world_to_base_tf_stamped_.transform.translation.z = 0.0;
    world_to_base_tf_stamped_.transform.rotation.x = 0.0;
    world_to_base_tf_stamped_.transform.rotation.y = 0.0;
    world_to_base_tf_stamped_.transform.rotation.z = 0.0;
    world_to_base_tf_stamped_.transform.rotation.w = 1.0;
    tf_br_.sendTransform(world_to_base_tf_stamped_);

    // Initialize the interactive marker
    initializeInteractiveMarker();
}

void Rx1Ik::make6DofMarker(visualization_msgs::InteractiveMarker &int_marker)
{
    int_marker.scale = 0.15;
    // Create a control for each degree of freedom

    // X-axis control (red)
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // Y-axis control (green)
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // Z-axis control (blue)
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    // Rotation around X-axis
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    // Rotation around Y-axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    // Rotation around Z-axis
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
}

void Rx1Ik::initializeInteractiveMarker()
{
    // Create an interactive marker for the end effector
    //visualization_msgs::InteractiveMarker int_marker;
    int_marker_r_.header.frame_id = chain_start_; //"torso_link";  
    int_marker_r_.name = "right_end_effector";
    int_marker_r_.description = "Right End Effector Control";

    
    int_marker_l_.header.frame_id = chain_start_;
    int_marker_l_.name = "left_end_effector";
    int_marker_l_.description = "Left End Effector Control";

    /*
    int_marker_b_.header.frame_id = "map";
    int_marker_b_.name = "base_end_effector";
    int_marker_b_.description = "Base End Effector Control";
    */

    // Create a 6-DOF control which allows moving and rotating along all axes
    make6DofMarker(int_marker_r_);
    make6DofMarker(int_marker_l_);
    //make6DofMarker(int_marker_b_);

    // Add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    marker_server_.insert(int_marker_r_, boost::bind(&Rx1Ik::markerRightCallback, this, _1));
    marker_server_.insert(int_marker_l_, boost::bind(&Rx1Ik::markerLeftCallback, this, _1));
    //marker_server_.insert(int_marker_b_, boost::bind(&Rx1Ik::markerBaseCallback, this, _1));

    // 'commit' changes and send to all clients
    marker_server_.applyChanges();
}

void Rx1Ik::markerRightCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
        // Convert the marker pose to a KDL::Frame
        KDL::Frame desired_pose;
        tf2::fromMsg(feedback->pose, desired_pose);
        
        // Solve IK
        KDL::JntArray result_joint_positions;
        if (ik_solver_r_ptr_->solveIK(desired_pose, result_joint_positions))
        {
            bool success = true;
            for (int i = 0; i < result_joint_positions.rows(); ++i)
            {
                if (abs(right_prev_joint_state_msg_.position[i]-result_joint_positions(i)) > max_angle_change_)
                    success = false;
            }

            if (success)
            {
                for (int i = 0; i < result_joint_positions.rows(); ++i)
                {
                    right_prev_joint_state_msg_.position[i] = right_prev_joint_state_msg_.position[i]*0.9 + result_joint_positions(i)*0.1;
                    //prev_joint_state_msg_.position[i] = result_joint_positions(i);
                }
                ROS_INFO("Succeed finding right arm IK solution");
            }
            else
            {
                ROS_INFO("Succeed finding right arm IK solution but ditch the result to reduce shake");
            }
        }
        else
        {
            ROS_WARN("Failed to find right arm IK solution");
        }
    }
}

void Rx1Ik::markerLeftCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
        // Convert the marker pose to a KDL::Frame
        KDL::Frame desired_pose;
        tf2::fromMsg(feedback->pose, desired_pose);
        
        // Solve IK
        KDL::JntArray result_joint_positions;
        if (ik_solver_l_ptr_->solveIK(desired_pose, result_joint_positions))
        {
            bool success = true;
            for (int i = 0; i < result_joint_positions.rows(); ++i)
            {
                if (abs(left_prev_joint_state_msg_.position[i]-result_joint_positions(i)) > max_angle_change_)
                    success = false;
            }

            if (success)
            {
                for (int i = 0; i < result_joint_positions.rows(); ++i)
                {
                    left_prev_joint_state_msg_.position[i] = left_prev_joint_state_msg_.position[i]*0.9 + result_joint_positions(i)*0.1;
                    //prev_joint_state_msg_.position[i] = result_joint_positions(i);
                }
                ROS_INFO("Succeed finding left arm IK solution");
            }
            else
            {
                ROS_INFO("Succeed finding left arm IK solution but ditch the result to reduce shake");
            }
        }
        else
        {
            ROS_WARN("Failed to find left arm IK solution");
        }
    }

}

/*
void Rx1Ik::markerBaseCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    ROS_INFO("Marker base call back");
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
        bool failed = false;
        
        // save the old right foot and left foot pose in world frame
        geometry_msgs::PoseStamped old_r_pose;
        geometry_msgs::PoseStamped old_l_pose;
        geometry_msgs::PoseStamped old_base_pose;
        getLinkPose("map", "r_tipLink", old_r_pose);
        getLinkPose("map", "l_tipLink", old_l_pose);
        getLinkPose("map", "base", old_base_pose);

        // send the new transform between the world and base frame
        geometry_msgs::TransformStamped tf_stamped = poseToTransformStamped(feedback->pose, "map", "base");
        world_to_base_tf_stamped_ = tf_stamped;        
        tf_br_.sendTransform(tf_stamped);

        // Get the new pose of left and right foot in the new base frame
        geometry_msgs::PoseStamped new_r_pose;
        geometry_msgs::PoseStamped new_l_pose;
        if (getPoseInNewFrame(old_r_pose, "base", new_r_pose))
        {
            ROS_INFO("Get new right foot pose in new base frame.");
        }
        else
        {
            failed = true;
        }
        if (getPoseInNewFrame(old_l_pose, "base", new_l_pose))
        {
            ROS_INFO("Get new left foot pose in new base frame.");
        }
        else
        {
            failed = true;
        }

        if (!failed)
        {
            // Convert the marker pose to a KDL::Frame
            KDL::Frame desired_r_pose;
            KDL::Frame desired_l_pose;
            tf2::fromMsg(new_r_pose.pose, desired_r_pose);
            tf2::fromMsg(new_l_pose.pose, desired_l_pose);

            // Solve IK
            KDL::JntArray result_joint_r_positions;
            KDL::JntArray result_joint_l_positions;
            if (ik_solver_r_ptr_->solveIK(desired_r_pose, result_joint_r_positions)
                && ik_solver_l_ptr_->solveIK(desired_l_pose, result_joint_l_positions))
            {
                for (int i = 0; i < 6; ++i)
                {
                    prev_joint_state_msg_.position[i] = result_joint_r_positions(i);
                    prev_joint_state_msg_.position[i+6] = result_joint_l_positions(i);
                }
                ROS_INFO("Suceed finding IK solution");
            }
            else
            {
                ROS_WARN("Failed to find IK solution");
                failed = true;
            }
        }

        if (failed)
        {
            // Restore the previous base pose in world frame
            geometry_msgs::TransformStamped tf_stamped = poseToTransformStamped(old_base_pose.pose, "map", "base");
            tf_br_.sendTransform(tf_stamped);
            world_to_base_tf_stamped_ = tf_stamped;        
        }
    }
}
*/

void Rx1Ik::rightGripperPoseCallback(const geometry_msgs::Pose& msg)
{
    // Convert the msg pose to a KDL::Frame
    KDL::Frame desired_pose;
    tf2::fromMsg(msg, desired_pose);
    
    // Solve IK
    KDL::JntArray result_joint_positions;
    if (ik_solver_r_ptr_->solveIK(desired_pose, result_joint_positions))
    {
        bool success = true;
        if ((ros::Time::now().toSec() - right_last_ik_time_) < tracking_timeout_) // if it's within tracking_timeout, then the angle change should be smaller than max_angle_change
        {
            for (int i = 0; i < result_joint_positions.rows(); ++i)
            {
                if (abs(right_prev_joint_state_msg_.position[i]-result_joint_positions(i)) > max_angle_change_)
                    success = false;
            }
        }

        if (success)
        {
            if ((ros::Time::now().toSec() - right_last_ik_time_) < tracking_timeout_)
            {
                for (int i = 0; i < result_joint_positions.rows(); ++i)
                {
                    right_prev_joint_state_msg_.position[i] = right_prev_joint_state_msg_.position[i]*0.9 + result_joint_positions(i)*0.1;
                    //prev_joint_state_msg_.position[i] = result_joint_positions(i);
                }
            }
            else
            {
                for (int i = 0; i < result_joint_positions.rows(); ++i)
                {
                    right_prev_joint_state_msg_.position[i] = result_joint_positions(i);
                }
            }

            right_last_ik_time_ = ros::Time::now().toSec();
            ROS_INFO("Succeed finding right IK solution");
        }
        else
        {
            ROS_INFO("Succeed finding right IK solution but ditch the result to reduce shake");
        }
    }
    else
    {
        ROS_WARN("Failed to find right IK solution");
    }
}

void Rx1Ik::leftGripperPoseCallback(const geometry_msgs::Pose& msg)
{
    // Convert the msg pose to a KDL::Frame
    KDL::Frame desired_pose;
    tf2::fromMsg(msg, desired_pose);
    
    // Solve IK
    KDL::JntArray result_joint_positions;
    if (ik_solver_r_ptr_->solveIK(desired_pose, result_joint_positions))
    {
        bool success = true;
        if ((ros::Time::now().toSec() - left_last_ik_time_) < tracking_timeout_) // if it's within tracking_timeout, then the angle change should be smaller than max_angle_change
        {
            for (int i = 0; i < result_joint_positions.rows(); ++i)
            {
                if (abs(left_prev_joint_state_msg_.position[i]-result_joint_positions(i)) > max_angle_change_)
                    success = false;
            }
        }

        if (success)
        {
            if ((ros::Time::now().toSec() - left_last_ik_time_) < tracking_timeout_)
            {
                for (int i = 0; i < result_joint_positions.rows(); ++i)
                {
                    left_prev_joint_state_msg_.position[i] = left_prev_joint_state_msg_.position[i]*0.9 + result_joint_positions(i)*0.1;
                    //prev_joint_state_msg_.position[i] = result_joint_positions(i);
                }
            }
            else
            {
                for (int i = 0; i < result_joint_positions.rows(); ++i)
                {
                    left_prev_joint_state_msg_.position[i] = result_joint_positions(i);
                }
            }

            left_last_ik_time_ = ros::Time::now().toSec();
            ROS_INFO("Succeed finding left IK solution");
        }
        else
        {
            ROS_INFO("Succeed finding left IK solution but ditch the result to reduce shake");
        }
    }
    else
    {
        ROS_WARN("Failed to find left IK solution");
    }
}
void Rx1Ik::spinOnce()
{
    ros::spinOnce();
    update();
}

void Rx1Ik::update()
{
    right_prev_joint_state_msg_.header.stamp = ros::Time::now();
    right_joint_state_pub_.publish(right_prev_joint_state_msg_);
    left_prev_joint_state_msg_.header.stamp = ros::Time::now();
    left_joint_state_pub_.publish(left_prev_joint_state_msg_);
  
    world_to_base_tf_stamped_.header.stamp = ros::Time::now();
    tf_br_.sendTransform(world_to_base_tf_stamped_);
    geometry_msgs::PoseStamped pose;
    if(getLinkPose(chain_start_, chain_r_end_, pose))
    {
        int_marker_r_.pose = pose.pose;
        marker_server_.insert(int_marker_r_);
        marker_server_.applyChanges();
        ROS_INFO("Marker right pose updated");
    }
    if(getLinkPose(chain_start_, chain_l_end_, pose))
    {
        int_marker_l_.pose = pose.pose;
        marker_server_.insert(int_marker_l_);
        marker_server_.applyChanges();
        ROS_INFO("Marker left pose updated");
    }
    /*
    if(getLinkPose("map", "base", pose))
    {
        int_marker_b_.pose = pose.pose;
        marker_server_.insert(int_marker_b_);
        marker_server_.applyChanges();
        ROS_INFO("Marker base pose updated");
    }
    */
}

void Rx1Ik::spin()
{
    ros::Rate rate(25);
    while(ros::ok())
    {
        spinOnce();
        rate.sleep();
    }
}


bool Rx1Ik::getLinkPose(const std::string frame, const std::string link, geometry_msgs::PoseStamped& pose)
{
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tf_buffer_.lookupTransform(frame, link, ros::Time(0), ros::Duration(3.0));
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = frame;
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation = transformStamped.transform.rotation;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false; 
    }
    return true; 
}

bool Rx1Ik::getPoseInNewFrame(const geometry_msgs::PoseStamped old_pose, const std::string new_frame, geometry_msgs::PoseStamped& new_pose)
{
    try {
        new_pose = tf_buffer_.transform(old_pose, new_frame, ros::Duration(1.0));
        //new_pose = tf_buffer_.transform(old_pose, new_frame);
    } 
    catch (tf2::TransformException &ex) {
        ROS_WARN("getPoseInNewFrame: %s", ex.what());
        return false; 
    }
    return true;
}

geometry_msgs::TransformStamped Rx1Ik::poseToTransformStamped(const geometry_msgs::Pose& pose, const std::string& frame_id, const std::string& child_frame_id) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;

    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = pose.position.z;
    transformStamped.transform.rotation = pose.orientation;

    return transformStamped;
}
