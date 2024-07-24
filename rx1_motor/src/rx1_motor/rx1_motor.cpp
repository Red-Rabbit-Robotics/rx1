#include "rx1_motor/rx1_motor.hpp"

#include <cmath>
#include <cassert>

namespace rx1_motor
{

constexpr std::array<int, 7> Rx1Motor::right_arm_servo_ids_;
constexpr std::array<int, 7> Rx1Motor::right_arm_servo_dirs_;
constexpr std::array<int, 7> Rx1Motor::right_arm_servo_gears_;
constexpr std::array<int, 7> Rx1Motor::left_arm_servo_ids_;
constexpr std::array<int, 7> Rx1Motor::left_arm_servo_dirs_;
constexpr std::array<int, 7> Rx1Motor::left_arm_servo_gears_;
constexpr std::array<int, 5> Rx1Motor::head_servo_ids_;
constexpr std::array<int, 5> Rx1Motor::head_servo_dirs_;
constexpr std::array<int, 5> Rx1Motor::head_servo_gears_;
constexpr std::array<int, 3> Rx1Motor::torso_servo_ids_;
constexpr std::array<int, 3> Rx1Motor::torso_servo_dirs_;
constexpr std::array<int, 3> Rx1Motor::torso_servo_gears_;
constexpr std::array<int, 6> Rx1Motor::right_hand_servo_ids_;
constexpr std::array<int, 6> Rx1Motor::right_hand_servo_default_;
constexpr std::array<int, 6> Rx1Motor::right_hand_servo_range_;
constexpr std::array<int, 6> Rx1Motor::left_hand_servo_ids_;
constexpr std::array<int, 6> Rx1Motor::left_hand_servo_default_;
constexpr std::array<int, 6> Rx1Motor::left_hand_servo_range_;

Rx1Motor::Rx1Motor(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
    : nh_(nh),
      priv_nh_(priv_nh)
{
    servo_port_ = "/dev/ttyUSB-arduino4.3";

    // Torso IK test
    /*
    auto angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, 0, 0); 
    ROS_INFO("torso ik result for pitch 0, roll 0 is: %f %f", angles[0], angles[1]);
    angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, 0.2, 0); 
    ROS_INFO("torso ik result for pitch 0.2, roll 0 is: %f %f", angles[0], angles[1]);
    angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, 0, 0.2); 
    ROS_INFO("torso ik result for pitch 0, roll 0.2 is: %f %f", angles[0], angles[1]);
    angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, 0.2, 0.2); 
    ROS_INFO("torso ik result for pitch 0.2, roll 0.2 is: %f %f", angles[0], angles[1]);
    */
    if (!sts_servo_.begin(1000000, servo_port_.c_str()))
    {
        ROS_ERROR("[RX1_MOTOR] Failed initialize sts servo!");
    }

    if (!scs_servo_.begin(1000000, servo_port_.c_str()))
    {
        ROS_ERROR("[RX1_MOTOR] Failed initialize scs servo!");
    }
    
    for(int i = 0; i < right_arm_servo_ids_.size(); i ++)
    {
        u8 id = right_arm_servo_ids_[i];
        sts_servo_.WritePosEx(id, 2048, 200, 20);
    }
    for(int i = 0; i < left_arm_servo_ids_.size(); i ++)
    {
        u8 id = left_arm_servo_ids_[i];
        sts_servo_.WritePosEx(id, 2048, 200, 20);
    }
    for(int i = 0; i < torso_servo_ids_.size(); i ++)
    {
        u8 id = torso_servo_ids_[i];
        sts_servo_.WritePosEx(id, 2048, 200, 20);
    }
    for(int i = 0; i < head_servo_ids_.size(); i ++)
    {
        u8 id = head_servo_ids_[i];
        
        if (i == 0 || i == 1 || i == 2)
        {
            sts_servo_.WritePosEx(id, 2048, 200, 20);
        }
        else
        {
            scs_servo_.WritePos(id, 512, 0, 100);
        }
    }

    last_spin_time_ = ros::Time::now();
}

Rx1Motor::~Rx1Motor()
{
    sts_servo_.end();
}

void Rx1Motor::spinOnce()
{
    ros::spinOnce();
    update();
}

void Rx1Motor::spin()
{
    ros::Rate rate(100);
    while(ros::ok())
    {
        spinOnce();
        rate.sleep();
        double actual_rate = 1.0 / (ros::Time::now() - last_spin_time_).toSec();
        last_spin_time_ = ros::Time::now();
        ROS_INFO("[RX1_MOTOR] actual rate is %f", actual_rate);
    }
}

void Rx1Motor::update()
{
}


void Rx1Motor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::vector<double> joint_positions = msg->position;

    std::vector<double> torso_joint_positions(3);
    std::vector<double> head_joint_positions(5);
    std::vector<double> right_arm_joint_positions(7);
    std::vector<double> left_arm_joint_positions(7);
   
    for (int i = 0; i < joint_positions.size(); i ++)
    {
        if (i < 3)
        {
            torso_joint_positions[i] = joint_positions[i];
        }
        else if (i < 8)
        {
            head_joint_positions[i-3] = joint_positions[i];
        }
        else if (i < 15)
        {
            right_arm_joint_positions[i-8] = joint_positions[i];
        }
        else
        {
            left_arm_joint_positions[i-15] = joint_positions[i];
        }
    }

    // Process the joint state information
    auto angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, torso_joint_positions[1], torso_joint_positions[2]); 
    torso_joint_positions[1] = angles[0];
    torso_joint_positions[2] = angles[1];
    motorCommand(torso_servo_ids_, torso_servo_dirs_, torso_servo_gears_, torso_joint_positions, TORSO_SPEED_, TORSO_ACC_);

    headMotorCommand(head_joint_positions);

    motorCommand(right_arm_servo_ids_, right_arm_servo_dirs_, right_arm_servo_gears_, right_arm_joint_positions, ARM_SPEED_, ARM_ACC_);
    motorCommand(left_arm_servo_ids_, left_arm_servo_dirs_, left_arm_servo_gears_, left_arm_joint_positions, ARM_SPEED_, ARM_ACC_);

}


void Rx1Motor::rightArmJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Access the joint state information
    std::vector<double> joint_positions = msg->position;

    ros::Time command_start_time = ros::Time::now(); 
    // Process the joint state information
    motorCommand(right_arm_servo_ids_, right_arm_servo_dirs_, right_arm_servo_gears_, joint_positions, ARM_SPEED_, ARM_ACC_);
    double time_spend = (ros::Time::now() - command_start_time).toSec();
    ROS_INFO("[RX1_MOTOR] right arm command time is %f sec", time_spend);
}

void Rx1Motor::leftArmJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Access the joint state information
    std::vector<double> joint_positions = msg->position;

    // Process the joint state information
    motorCommand(left_arm_servo_ids_, left_arm_servo_dirs_, left_arm_servo_gears_, joint_positions, ARM_SPEED_, ARM_ACC_);
}

void Rx1Motor::torsoJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Access the joint state information
    std::vector<double> joint_positions = msg->position;
    
    auto angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, joint_positions[2], joint_positions[1]); 

    joint_positions[1] = angles[0];
    joint_positions[2] = angles[1];

    motorCommand(torso_servo_ids_, torso_servo_dirs_, torso_servo_gears_, joint_positions, TORSO_SPEED_, TORSO_ACC_);
}

std::array<double, 2> Rx1Motor::torsoIk(double d, double L1, double h1, double h2, double roll, double pitch)
{
    // obtained from https://github.com/rocketman123456/ros2_ws/blob/master/src/humanoid_motor_control_cpp/src/humanoid_control/kinematic/ankle_ik.cpp
    double cx = cos(pitch);
    double sx = sin(pitch);
    double cy = cos(roll);
    double sy = sin(roll);

    double AL = - L1 * L1 * cy + L1 * d * sx * sy;
    double BL = - L1 * L1 * sy + L1 * h1 - L1 * d * sx * cy;
    double CL = -(L1 * L1 + d * d - d * d *cx - L1 * h1 * sy - d * h1 * sx * cy);

    double LenL = sqrt(AL * AL + BL * BL);

    double AR = - L1 * L1 * cy - L1 * d * sx * sy;
    double BR = - L1 * L1 * sy + L1 * h2 + L1 * d * sx * cy;
    double CR = -(L1 * L1 + d * d - d * d *cx - L1 * h2 * sy + d * h2 * sx * cy);

    double LenR = sqrt(AR * AR + BR * BR);

    if (LenL <= abs(CL) || LenR <= abs(CR))
    {
        return {0.0, 0.0};
    }
    else
    {
        double tL_1 = asin(CL / LenL) - asin(AL / LenL);
        double tL_2 = asin(CL / LenL) + acos(BL / LenL);

        double tR_1 = asin(CR / LenR) - asin(AR / LenR);
        double tR_2 = asin(CR / LenR) + acos(BR / LenR);

        assert(fabs(tL_1 - tL_2) < 1e-3 && "tL_1 - tL_2 > 1e-3");
        assert(fabs(tR_1 - tR_2) < 1e-3 && "tR_1 - tR_2 > 1e-3");

        return {tL_1, tR_1};
    }
}

void Rx1Motor::headJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Access the joint state information
    std::vector<double> joint_positions = msg->position;

    // Process the joint state information
    //motorCommand(head_servo_ids_, head_servo_dirs_, head_servo_gears_, joint_positions, HEAD_SPEED_, HEAD_ACC_);
    headMotorCommand(joint_positions);
}

void Rx1Motor::headMotorCommand(const std::vector<double>& joint_positions)
{   
    u8 id;
    s16 pos;
    u16 speed = HEAD_SPEED_;
    u8 acc = HEAD_ACC_;

    for (int i = 0; i < joint_positions.size(); i ++)
    {
        id = head_servo_ids_[i]; 
        if (i == 0 || i == 1 || i == 2) //neck
        {
            pos = joint_positions[i]/3.14*2048*head_servo_dirs_[i]*head_servo_gears_[i] + 2048;
            sts_servo_.WritePosEx(id, pos, speed, acc);
        }
        else // ear
        {
            pos = joint_positions[i]/3.14*512*head_servo_dirs_[i]*head_servo_gears_[i] + 512;
            scs_servo_.WritePos(id, pos, 0, 0); // id, pos, time, speed
        }
    }

}

void Rx1Motor::rightGripperCallback(const std_msgs::Float32ConstPtr& msg)
{
    ros::Time command_start_time = ros::Time::now(); 

    double grip_ratio = msg->data;

    int length = right_hand_servo_ids_.size();
    u8 id;
    s16 pos;
    u16 speed = HAND_SPEED_;
    u8 acc = HAND_ACC_;    
    
    for (int i = 0; i < length; i ++)
    {
        id = right_hand_servo_ids_[i];
        pos = right_hand_servo_default_[i] + grip_ratio * right_hand_servo_range_[i];
        
        if (i == 1 || i == 2) // thumb and index fingers are sts servo, others are scs servos
        {
            sts_servo_.WritePosEx(id, pos, speed, acc);
        }
        else if (i == 3)
        {
            scs_servo_.WritePos(id, pos, 0, 400); // id, pos, time, speed
        }
        else if (i == 4)
        {
            scs_servo_.WritePos(id, pos, 0, 300); // id, pos, time, speed
        }
        else if (i == 5)
        {
            scs_servo_.WritePos(id, pos, 0, 200); // id, pos, time, speed
        }
    }

    // Thumb yaw
    scs_servo_.WritePos(right_hand_servo_ids_[0], 200, 0, 400); // id, pos, time, speed
   
    double time_spend = (ros::Time::now() - command_start_time).toSec();
    ROS_INFO("[RX1_MOTOR] right hand command time is %f sec", time_spend);
}

void Rx1Motor::leftGripperCallback(const std_msgs::Float32::ConstPtr& msg)
{
    double grip_ratio = msg->data;

    int length = left_hand_servo_ids_.size();
    u8 id;
    s16 pos;
    u16 speed = HAND_SPEED_;
    u8 acc = HAND_ACC_;
    
    for (int i = 0; i < length; i ++)
    {
        id = left_hand_servo_ids_[i];
        pos = left_hand_servo_default_[i] + grip_ratio * left_hand_servo_range_[i];
        if (i == 1 || i == 2) // thumb and index fingers are sts servo, others are scs servos
        {
            sts_servo_.WritePosEx(id, pos, speed, acc);
        }
        else if (i == 3)
        {
            scs_servo_.WritePos(id, pos, 0, 400); // id, pos, time, speed
        }
        else if (i == 4)
        {
            scs_servo_.WritePos(id, pos, 0, 300); // id, pos, time, speed
        }
        else if (i == 5)
        {
            scs_servo_.WritePos(id, pos, 0, 200); // id, pos, time, speed
        }
    }

    // Thumb yaw
    scs_servo_.WritePos(left_hand_servo_ids_[0], 512, 0, 400); // id, pos, time, speed

}

template<size_t N>
void Rx1Motor::motorCommand(const std::array<int, N>& joint_ids,
                            const std::array<int, N>& joint_dirs,
                            const std::array<int, N>& joint_gears,
                            const std::vector<double>& joint_angles, 
                            const int speed, 
                            const int acc)
{
    int length = joint_ids.size();
    u8 *ids = (u8 *)malloc(length * sizeof(u8));
    s16 *pos = (s16 *)malloc(length * sizeof(s16));
    u16 *speeds = (u16* )malloc(length * sizeof(u16));
    u8 *accs = (u8 *)malloc(length * sizeof(u8));
 
    if (ids == NULL || pos == NULL || speeds == NULL || accs == NULL)
    {
        ROS_WARN("[RX1_MOTOR] Motor command memory allocation failed!");
    }
    /*
    u8 ids[7];
    s16 pos[7];
    u16 speeds[7];
    u8 accs[7];
    for (int i = 0; i < 7; i ++)
    {
        ids[i] = sts_servo_ids_[i];
        accs[i] = acc;
        speeds[i] = speed;
        pos[i] = joint_angles[i]/3.14*2048*sts_servo_dirs_[i]*sts_servo_gears_[i] + 2048;
    }
    */

    for (int i = 0; i < length; i ++)
    {
        ids[i] = joint_ids[i];
        accs[i] = acc*joint_gears[i];
        speeds[i] = speed*joint_gears[i];
        pos[i] = joint_angles[i]/3.14*2048*joint_dirs[i]*joint_gears[i] + 2048;
    
        // temporary change: make forearm faster
        if (i >= 3)
        {
            accs[i] = accs[i] * 10;
            speeds[i] = 0;
        }
    }
    sts_servo_.SyncWritePosEx(ids, length, pos, speeds, accs);

    free(ids);
    free(pos);
    free(speeds);
    free(accs);
}

} // namespace rx1_motor
