#include "rx1_motor/rx1_motor.hpp"

#include <cassert>
#include <cmath>
#include <memory>

namespace rx1_motor
{

using std::placeholders::_1;

Rx1Motor::Rx1Motor()
: rclcpp::Node("rx1_motor_node")
{
  servo_port_ = this->declare_parameter<std::string>("servo_port", "/dev/ttyUSB-arduino4.3");

  joint_state_sub_ = this->create_subscription<JointState>(
    "/command_joint_states", 10, std::bind(&Rx1Motor::jointStateCallback, this, _1));
  right_arm_joint_state_sub_ = this->create_subscription<JointState>(
    "/right_arm_joint_states", 10, std::bind(&Rx1Motor::rightArmJointStateCallback, this, _1));
  left_arm_joint_state_sub_ = this->create_subscription<JointState>(
    "/left_arm_joint_states", 10, std::bind(&Rx1Motor::leftArmJointStateCallback, this, _1));
  torso_joint_state_sub_ = this->create_subscription<JointState>(
    "/torso_joint_states", 10, std::bind(&Rx1Motor::torsoJointStateCallback, this, _1));
  head_joint_state_sub_ = this->create_subscription<JointState>(
    "/head_joint_states", 10, std::bind(&Rx1Motor::headJointStateCallback, this, _1));
  right_gripper_sub_ = this->create_subscription<Float32>(
    "/right_gripper", 10, std::bind(&Rx1Motor::rightGripperCallback, this, _1));
  left_gripper_sub_ = this->create_subscription<Float32>(
    "/left_gripper", 10, std::bind(&Rx1Motor::leftGripperCallback, this, _1));

  if (!sts_servo_.begin(1000000, servo_port_.c_str())) {
    RCLCPP_ERROR(get_logger(), "[RX1_MOTOR] Failed initialize sts servo port %s!", servo_port_.c_str());
  }

  if (!scs_servo_.begin(1000000, servo_port_.c_str())) {
    RCLCPP_ERROR(get_logger(), "[RX1_MOTOR] Failed initialize scs servo port %s!", servo_port_.c_str());
  }

  for (size_t i = 0; i < right_arm_servo_ids_.size(); ++i) {
    const u8 id = static_cast<u8>(right_arm_servo_ids_[i]);
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }
  for (size_t i = 0; i < left_arm_servo_ids_.size(); ++i) {
    const u8 id = static_cast<u8>(left_arm_servo_ids_[i]);
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }
  for (size_t i = 0; i < torso_servo_ids_.size(); ++i) {
    const u8 id = static_cast<u8>(torso_servo_ids_[i]);
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }
  for (size_t i = 0; i < head_servo_ids_.size(); ++i) {
    const u8 id = static_cast<u8>(head_servo_ids_[i]);
    if (i == 1 || i == 0) {
      sts_servo_.WritePosEx(id, 2048, 200, 20);
    } else if (i == 2) {
      sts_servo_.WritePosEx(id, 2000, 200, 20);
    } else {
      scs_servo_.WritePos(id, 512, 0, 100);
    }
  }

  last_spin_time_ = this->now();
}

Rx1Motor::~Rx1Motor()
{
  sts_servo_.end();
}

void Rx1Motor::spin()
{
  rclcpp::WallRate rate(100.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    update();
    rate.sleep();

    const auto now = this->now();
    const auto delta = now - last_spin_time_;
    const double seconds = delta.seconds();
    if (seconds > 0.0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "[RX1_MOTOR] actual rate is %f", 1.0 / seconds);
    }
    last_spin_time_ = now;
  }
}

void Rx1Motor::update()
{
}

void Rx1Motor::jointStateCallback(const JointState::SharedPtr msg)
{
  std::vector<double> torso_joint_positions(3);
  std::vector<double> head_joint_positions(5);
  std::vector<double> right_arm_joint_positions(7);
  std::vector<double> left_arm_joint_positions(7);

  for (size_t i = 0; i < msg->position.size(); ++i) {
    if (i < 3) {
      torso_joint_positions[i] = msg->position[i];
    } else if (i < 8) {
      head_joint_positions[i - 3] = msg->position[i];
    } else if (i < 15) {
      right_arm_joint_positions[i - 8] = msg->position[i];
    } else if (i < 22) {
      left_arm_joint_positions[i - 15] = msg->position[i];
    }
  }

  const auto angles = torsoIk(
    TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, torso_joint_positions[1], torso_joint_positions[2]);
  torso_joint_positions[1] = angles[0];
  torso_joint_positions[2] = angles[1];

  std::vector<double> torso_speeds(torso_servo_ids_.size(), TORSO_SPEED_);
  std::vector<double> torso_accs(torso_servo_ids_.size(), TORSO_ACC_);
  motorCommand(
    torso_servo_ids_, torso_servo_dirs_, torso_servo_gears_, torso_joint_positions,
    torso_speeds, torso_accs);

  std::vector<double> head_speeds(head_servo_ids_.size(), HEAD_SPEED_);
  std::vector<double> head_accs(head_servo_ids_.size(), HEAD_ACC_);
  headMotorCommand(head_joint_positions, head_speeds, head_accs);

  std::vector<double> arm_speeds(right_arm_servo_ids_.size(), ARM_SPEED_);
  std::vector<double> arm_accs(right_arm_servo_ids_.size(), ARM_ACC_);
  motorCommand(
    right_arm_servo_ids_, right_arm_servo_dirs_, right_arm_servo_gears_,
    right_arm_joint_positions, arm_speeds, arm_accs);
  motorCommand(
    left_arm_servo_ids_, left_arm_servo_dirs_, left_arm_servo_gears_,
    left_arm_joint_positions, arm_speeds, arm_accs);
}

void Rx1Motor::rightArmJointStateCallback(const JointState::SharedPtr msg)
{
  std::vector<double> arm_speeds(right_arm_servo_ids_.size(), ARM_SPEED_);
  std::vector<double> arm_accs(right_arm_servo_ids_.size(), ARM_ACC_);
  motorCommand(
    right_arm_servo_ids_, right_arm_servo_dirs_, right_arm_servo_gears_,
    msg->position, arm_speeds, arm_accs);
}

void Rx1Motor::leftArmJointStateCallback(const JointState::SharedPtr msg)
{
  std::vector<double> arm_speeds(left_arm_servo_ids_.size(), ARM_SPEED_);
  std::vector<double> arm_accs(left_arm_servo_ids_.size(), ARM_ACC_);
  motorCommand(
    left_arm_servo_ids_, left_arm_servo_dirs_, left_arm_servo_gears_,
    msg->position, arm_speeds, arm_accs);
}

void Rx1Motor::torsoJointStateCallback(const JointState::SharedPtr msg)
{
  auto joint_positions = msg->position;
  const auto angles = torsoIk(
    TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, joint_positions[2], joint_positions[1]);

  joint_positions[1] = angles[0];
  joint_positions[2] = angles[1];

  std::vector<double> torso_speeds(torso_servo_ids_.size(), TORSO_SPEED_);
  std::vector<double> torso_accs(torso_servo_ids_.size(), TORSO_ACC_);

  motorCommand(
    torso_servo_ids_, torso_servo_dirs_, torso_servo_gears_, joint_positions,
    torso_speeds, torso_accs);
}

std::array<double, 2> Rx1Motor::torsoIk(
  double d, double l1, double h1, double h2, double roll, double pitch)
{
  const double cx = std::cos(pitch);
  const double sx = std::sin(pitch);
  const double cy = std::cos(roll);
  const double sy = std::sin(roll);

  const double al = -l1 * l1 * cy + l1 * d * sx * sy;
  const double bl = -l1 * l1 * sy + l1 * h1 - l1 * d * sx * cy;
  const double cl = -(l1 * l1 + d * d - d * d * cx - l1 * h1 * sy - d * h1 * sx * cy);
  const double len_l = std::sqrt(al * al + bl * bl);

  const double ar = -l1 * l1 * cy - l1 * d * sx * sy;
  const double br = -l1 * l1 * sy + l1 * h2 + l1 * d * sx * cy;
  const double cr = -(l1 * l1 + d * d - d * d * cx - l1 * h2 * sy + d * h2 * sx * cy);
  const double len_r = std::sqrt(ar * ar + br * br);

  if (len_l <= std::abs(cl) || len_r <= std::abs(cr)) {
    return {0.0, 0.0};
  }

  const double tl_1 = std::asin(cl / len_l) - std::asin(al / len_l);
  const double tl_2 = std::asin(cl / len_l) + std::acos(bl / len_l);
  const double tr_1 = std::asin(cr / len_r) - std::asin(ar / len_r);
  const double tr_2 = std::asin(cr / len_r) + std::acos(br / len_r);

  assert(std::fabs(tl_1 - tl_2) < 1e-3);
  assert(std::fabs(tr_1 - tr_2) < 1e-3);

  return {tl_1, tr_1};
}

void Rx1Motor::headJointStateCallback(const JointState::SharedPtr msg)
{
  std::vector<double> joint_speeds(head_servo_ids_.size(), HEAD_SPEED_);
  std::vector<double> joint_accs(head_servo_ids_.size(), HEAD_ACC_);
  headMotorCommand(msg->position, joint_speeds, joint_accs);
}

void Rx1Motor::rightGripperCallback(const Float32::SharedPtr msg)
{
  const double grip_ratio = msg->data;
  const int length = static_cast<int>(right_hand_servo_ids_.size());
  const u16 speed = static_cast<u16>(HAND_SPEED_ * speed_const_);
  const u8 acc = static_cast<u8>(HAND_ACC_ * acc_const_);

  for (int i = 0; i < length; ++i) {
    const u8 id = static_cast<u8>(right_hand_servo_ids_[i]);
    const s16 pos = static_cast<s16>(right_hand_servo_default_[i] + grip_ratio * right_hand_servo_range_[i]);

    if (i == 1 || i == 2) {
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else if (i == 3) {
      scs_servo_.WritePos(id, pos, 0, 400);
    } else if (i == 4) {
      scs_servo_.WritePos(id, pos, 0, 300);
    } else if (i == 5) {
      scs_servo_.WritePos(id, pos, 0, 200);
    }
  }

  scs_servo_.WritePos(right_hand_servo_ids_[0], 200, 0, 400);
}

void Rx1Motor::leftGripperCallback(const Float32::SharedPtr msg)
{
  const double grip_ratio = msg->data;
  const int length = static_cast<int>(left_hand_servo_ids_.size());
  const u16 speed = static_cast<u16>(HAND_SPEED_ * speed_const_);
  const u8 acc = static_cast<u8>(HAND_ACC_ * acc_const_);

  for (int i = 0; i < length; ++i) {
    const u8 id = static_cast<u8>(left_hand_servo_ids_[i]);
    const s16 pos = static_cast<s16>(left_hand_servo_default_[i] + grip_ratio * left_hand_servo_range_[i]);

    if (i == 1 || i == 2) {
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else if (i == 3) {
      scs_servo_.WritePos(id, pos, 0, 400);
    } else if (i == 4) {
      scs_servo_.WritePos(id, pos, 0, 300);
    } else if (i == 5) {
      scs_servo_.WritePos(id, pos, 0, 200);
    }
  }

  scs_servo_.WritePos(left_hand_servo_ids_[0], 512, 0, 400);
}

void Rx1Motor::headMotorCommand(
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs)
{
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    const u8 id = static_cast<u8>(head_servo_ids_[i]);
    const u16 speed = static_cast<u16>(joint_speeds[i] * speed_const_);
    const u8 acc = static_cast<u8>(joint_accs[i] * acc_const_);

    if (i <= 2) {
      const s16 pos = static_cast<s16>(
        joint_positions[i] / 3.14 * 2048 * head_servo_dirs_[i] * head_servo_gears_[i] + 2048);
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else {
      const s16 pos = static_cast<s16>(
        joint_positions[i] / 3.14 * 512 * head_servo_dirs_[i] * head_servo_gears_[i] + 512);
      scs_servo_.WritePos(id, pos, 0, 0);
    }
  }
}

template<size_t N>
void Rx1Motor::motorCommand(
  const std::array<int, N> & joint_ids,
  const std::array<int, N> & joint_dirs,
  const std::array<int, N> & joint_gears,
  const std::vector<double> & joint_angles,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs)
{
  const int length = static_cast<int>(joint_ids.size());
  auto ids = std::make_unique<u8[]>(length);
  auto pos = std::make_unique<s16[]>(length);
  auto speeds = std::make_unique<u16[]>(length);
  auto accs = std::make_unique<u8[]>(length);

  for (int i = 0; i < length; ++i) {
    ids[i] = static_cast<u8>(joint_ids[i]);
    speeds[i] = static_cast<u16>(joint_speeds[i] * joint_gears[i] * speed_const_);
    accs[i] = static_cast<u8>(joint_accs[i] * joint_gears[i] * acc_const_);
    pos[i] = static_cast<s16>(joint_angles[i] / 3.14 * 2048 * joint_dirs[i] * joint_gears[i] + 2048);

    if (i >= 3) {
      speeds[i] = 0;
      accs[i] = static_cast<u8>(accs[i] * 10);
    }
  }

  sts_servo_.SyncWritePosEx(ids.get(), length, pos.get(), speeds.get(), accs.get());
}

}  // namespace rx1_motor
