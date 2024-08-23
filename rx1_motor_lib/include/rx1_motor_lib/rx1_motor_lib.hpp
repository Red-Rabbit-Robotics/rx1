#ifndef RX1_MOTOR_LIB_H
#define RX1_MOTOR_LIB_H

#include "feetech_lib/SMS_STS.h"
#include "feetech_lib/SCSCL.h"

#include <string>
#include <vector>
#include <array>

namespace rx1_motor_lib
{

struct MotorConfig {
   int id;
   int dir;
   int gear;
};

class Rx1MotorLib
{
public:
    Rx1MotorLib();
    ~Rx1MotorLib();

    void initializeServos(const std::string& servo_port);
    void headMotorCommand(const std::vector<double>& joint_positions,
                          const std::vector<double>& joint_speeds,
                          const std::vector<double>& joint_accs);

    void leftArmMotorCommand(const std::vector<double>& joint_positions,
                          const std::vector<double>& joint_speeds,
                          const std::vector<double>& joint_accs);
    
    void rightArmMotorCommand(const std::vector<double>& joint_positions,
                          const std::vector<double>& joint_speeds,
                          const std::vector<double>& joint_accs);

    void torsoMotorCommand(const std::vector<double>& joint_positions,
                          const std::vector<double>& joint_speeds,
                          const std::vector<double>& joint_accs);

    void leftGripperMotorCommand(const double grip_ratio);
    void rightGripperMotorCommand(const double grip_ratio);

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

    std::array<double, 2> torsoIk(double d, double L1, double h1, double h2, double roll, double pitch);
 
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

    static constexpr double HAND_SPEED_ = 0; // maximum
    static constexpr double HAND_ACC_ = 15; //100;

    // Based on Feetech manual, 50step/s = 0.732 RPM
    // then 1 step /s = 0.00153232 rad /s
    // 1/ 0.00153232 = 652.6051999582332, multiple this value to turn rad/s to Feetech speed
    const double SPEED_CONST_ = 652.6051999582332;
    // Based on Feetech manual, 1 unit of acc is 100 step/s^2
    // thus we can multiple this value to turn rad/s^2 to Feetech acc
    const double ACC_CONST_ = 6.526051999582332;

}; 

} // namespace rx_motor_lib

#endif // RX!_MOTOR_LIB_H
