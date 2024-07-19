#ifndef RX1_MOTOR_H
#define RX1_MOTOR_H

#include "feetech_lib/SMSBL.h"
#include "feetech_lib/SCSCL.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

#include <string>
#include <vector>

namespace rx1_motor
{

class Rx1Motor
{
public:
    Rx1Motor(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);
    ~Rx1Motor();

    void spinOnce();
    void spin();
    void update();

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;

private:
    std::string servo_port_;
    SMSBL sts_servo_;
    SCSCL scs_servo_;

    static constexpr std::array<int, 7> right_arm_servo_ids_ = {11, 12, 13, 14, 15, 16, 17};
    static constexpr std::array<int, 7> right_arm_servo_dirs_ = {-1, -1, 1, 1, 1, 1, -1};
    static constexpr std::array<int, 7> right_arm_servo_gears_ = {3, 3, 3, 3, 1, 1, 1};

    static constexpr std::array<int, 7> left_arm_servo_ids_ = {21, 22, 23, 24, 25, 26, 27};
    static constexpr std::array<int, 7> left_arm_servo_dirs_ = {-1, -1, 1, -1, 1, -1, -1};
    static constexpr std::array<int, 7> left_arm_servo_gears_ = {3, 3, 3, 3, 1, 1, 1};

    static constexpr std::array<int, 5> head_servo_ids_ = {4, 5, 6, 7, 8};
    static constexpr std::array<int, 5> head_servo_dirs_ = {-1, -1, -1, 1, -1};
    static constexpr std::array<int, 5> head_servo_gears_ = {1, 1, 1, 1, 1};
   
    static constexpr std::array<int, 3> torso_servo_ids_ = {1, 2, 3};
    static constexpr std::array<int, 3> torso_servo_dirs_ = {-1, 1, -1};
    static constexpr std::array<int, 3> torso_servo_gears_ = {3, 3, 3};

    static constexpr std::array<int, 6> right_hand_servo_ids_ = {31, 32, 33, 34, 35, 36};
    static constexpr std::array<int, 6> right_hand_servo_default_ = {200, 2400, 2300, 420, 600, 440};
    static constexpr std::array<int, 6> right_hand_servo_range_ = {0, -300, -400, 200, -200, 200};

    static constexpr std::array<int, 6> left_hand_servo_ids_ = {41, 42, 43, 44, 45, 46};
    static constexpr std::array<int, 6> left_hand_servo_default_ = {512, 2400, 2400, 440, 600, 440};
    static constexpr std::array<int, 6> left_hand_servo_range_ = {0, -100, -400, 200, -200, 200};

    ros::Subscriber joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/command_joint_states", 10, &Rx1Motor::jointStateCallback, this);
    ros::Subscriber right_arm_joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/right_arm_joint_states", 100, &Rx1Motor::rightArmJointStateCallback, this);
    ros::Subscriber left_arm_joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/left_arm_joint_states", 10, &Rx1Motor::leftArmJointStateCallback, this);
    ros::Subscriber torso_joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/torso_joint_states", 10, &Rx1Motor::torsoJointStateCallback, this);
    ros::Subscriber head_joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/head_joint_states", 10, &Rx1Motor::headJointStateCallback, this);
    ros::Subscriber right_gripper_sub_ = nh_.subscribe<std_msgs::Float32>("/right_gripper", 100, &Rx1Motor::rightGripperCallback, this);
    ros::Subscriber left_gripper_sub_ = nh_.subscribe<std_msgs::Float32>("/left_gripper", 10, &Rx1Motor::leftGripperCallback, this);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void rightArmJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftArmJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void torsoJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void headJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void leftGripperCallback(const std_msgs::Float32::ConstPtr& msg);
    void rightGripperCallback(const std_msgs::Float32::ConstPtr& msg);

    std::array<double, 2> torsoIk(double d, double L1, double h1, double h2, double roll, double pitch);
   
    void headMotorCommand(const std::vector<double>& joint_positinos);

    template<size_t N>
    void motorCommand(const std::array<int, N>& joint_ids,
                                const std::array<int, N>& joint_dirs,
                                const std::array<int, N>& joint_gears,
                                const std::vector<double>& joint_angles, 
                                const int speed=100, 
                                const int acc= 20);

    static constexpr double TORSO_D_ = 0.0865;
    static constexpr double TORSO_L1_ = 0.05;
    static constexpr double TORSO_H1_ = 0.11;
    static constexpr double TORSO_H2_ = 0.11;

    static constexpr int TORSO_SPEED_ = 500;
    static constexpr int TORSO_ACC_ = 10;
    static constexpr int ARM_SPEED_ = 700;
    static constexpr int ARM_ACC_ = 20;
    static constexpr int HEAD_SPEED_ = 500;
    static constexpr int HEAD_ACC_ = 50;
    static constexpr int HAND_SPEED_ = 0;
    static constexpr int HAND_ACC_ = 100;

    ros::Time last_spin_time_;
};

} // namespace rx1_motor

#endif // RX1_MOTOR_H
