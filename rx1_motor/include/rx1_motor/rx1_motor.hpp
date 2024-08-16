#ifndef RX1_MOTOR_H
#define RX1_MOTOR_H

#include "feetech_lib/SMS_STS.h"
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
    SMS_STS sts_servo_;
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
    static constexpr std::array<int, 6> left_hand_servo_default_ = {512, 1700, 1700, 650, 420, 630};
    static constexpr std::array<int, 6> left_hand_servo_range_ = {0, 300, 500, -210, 210, -240};

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
   
    void headMotorCommand(const std::vector<double>& joint_positinos,
                          const std::vector<double>& joint_speeds,
                          const std::vector<double>& joint_accs);

    template<size_t N>
    void motorCommand(const std::array<int, N>& joint_ids,
                                const std::array<int, N>& joint_dirs,
                                const std::array<int, N>& joint_gears,
                                const std::vector<double>& joint_angles, 
                                const std::vector<double>& joint_speeds, 
                                const std::vector<double>& joint_accs);

    static constexpr double TORSO_D_ = 0.0865;
    static constexpr double TORSO_L1_ = 0.05;
    static constexpr double TORSO_H1_ = 0.11;
    static constexpr double TORSO_H2_ = 0.11;

    static constexpr double TORSO_SPEED_ = 1.0;// 500;
    static constexpr double TORSO_ACC_ = 1.5; //10;
    static constexpr double ARM_SPEED_ = 1.0; //700;
    static constexpr double ARM_ACC_ = 3.0; //20;
    static constexpr double HEAD_SPEED_ = 1.0; //500;
    static constexpr double HEAD_ACC_ = 7.0; //50;
    static constexpr double HAND_SPEED_ = 0; // maximum
    static constexpr double HAND_ACC_ = 15; //100;

    // Based on Feetech manual, 50step/s = 0.732 RPM
    // then 1 step /s = 0.00153232 rad /s
    // 1/ 0.00153232 = 652.6051999582332, multiple this value to turn rad/s to Feetech speed
    const double SPEED_CONST_ = 652.6051999582332;
    // Based on Feetech manual, 1 unit of acc is 100 step/s^2
    // thus we can multiple this value to turn rad/s^2 to Feetech acc
    const double ACC_CONST_ = 6.526051999582332;

    ros::Time last_spin_time_;
};

} // namespace rx1_motor

#endif // RX1_MOTOR_H
