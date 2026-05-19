#ifndef RX1_MOTOR_H
#define RX1_MOTOR_H

#include "feetech_lib/SCSCL.h"
#include "feetech_lib/SMS_STS.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

#include <array>
#include <string>
#include <vector>

namespace rx1_motor
{

class Rx1Motor : public rclcpp::Node
{
public:
  Rx1Motor();
  ~Rx1Motor() override;

  void spin();
  void update();

private:
  using JointState = sensor_msgs::msg::JointState;
  using Float32 = std_msgs::msg::Float32;

  void jointStateCallback(const JointState::SharedPtr msg);
  void rightArmJointStateCallback(const JointState::SharedPtr msg);
  void leftArmJointStateCallback(const JointState::SharedPtr msg);
  void torsoJointStateCallback(const JointState::SharedPtr msg);
  void headJointStateCallback(const JointState::SharedPtr msg);
  void leftGripperCallback(const Float32::SharedPtr msg);
  void rightGripperCallback(const Float32::SharedPtr msg);

  std::array<double, 2> torsoIk(
    double d, double l1, double h1, double h2, double roll, double pitch);

  void headMotorCommand(
    const std::vector<double> & joint_positions,
    const std::vector<double> & joint_speeds,
    const std::vector<double> & joint_accs);

  template<size_t N>
  void motorCommand(
    const std::array<int, N> & joint_ids,
    const std::array<int, N> & joint_dirs,
    const std::array<int, N> & joint_gears,
    const std::vector<double> & joint_angles,
    const std::vector<double> & joint_speeds,
    const std::vector<double> & joint_accs);

  std::string servo_port_;
  SMS_STS sts_servo_;
  SCSCL scs_servo_;

  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<JointState>::SharedPtr right_arm_joint_state_sub_;
  rclcpp::Subscription<JointState>::SharedPtr left_arm_joint_state_sub_;
  rclcpp::Subscription<JointState>::SharedPtr torso_joint_state_sub_;
  rclcpp::Subscription<JointState>::SharedPtr head_joint_state_sub_;
  rclcpp::Subscription<Float32>::SharedPtr right_gripper_sub_;
  rclcpp::Subscription<Float32>::SharedPtr left_gripper_sub_;

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
  static constexpr std::array<int, 6> right_hand_servo_default_ = {200, 2350, 2300, 420, 600, 440};
  static constexpr std::array<int, 6> right_hand_servo_range_ = {0, -320, -420, 220, -220, 220};

  static constexpr std::array<int, 6> left_hand_servo_ids_ = {41, 42, 43, 44, 45, 46};
  static constexpr std::array<int, 6> left_hand_servo_default_ = {512, 1700, 1700, 650, 420, 630};
  static constexpr std::array<int, 6> left_hand_servo_range_ = {0, 350, 550, -260, 260, -300};

  static constexpr double TORSO_D_ = 0.0865;
  static constexpr double TORSO_L1_ = 0.05;
  static constexpr double TORSO_H1_ = 0.11;
  static constexpr double TORSO_H2_ = 0.11;

  static constexpr double TORSO_SPEED_ = 1.0;
  static constexpr double TORSO_ACC_ = 1.5;
  static constexpr double ARM_SPEED_ = 1.0;
  static constexpr double ARM_ACC_ = 3.0;
  static constexpr double HEAD_SPEED_ = 1.0;
  static constexpr double HEAD_ACC_ = 7.0;
  static constexpr double HAND_SPEED_ = 0.0;
  static constexpr double HAND_ACC_ = 15.0;

  const double speed_const_ = 652.6051999582332;
  const double acc_const_ = 6.526051999582332;

  rclcpp::Time last_spin_time_;
};

}  // namespace rx1_motor

#endif  // RX1_MOTOR_H
