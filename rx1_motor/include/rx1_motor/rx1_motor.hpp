#ifndef RX1_MOTOR_H
#define RX1_MOTOR_H

#include "feetech_lib/SMSBL.h"
#include "feetech_lib/SCSCL.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

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

    //std::vector<int> sts_servo_ids_ = {0, 1, 2, 3, 4, 5};
    std::vector<int> sts_servo_ids_ = {11, 12, 13, 14, 15, 16, 17};
    std::vector<int> sts_servo_dirs_ = {-1, -1, 1, 1, 1, 1, -1};
    std::vector<int> sts_servo_gears_ = {3, 2, 2, 3, 1, 1, 1};

    ros::Subscriber joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &Rx1Motor::jointStateCallback, this);


    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void motorCommand(const std::vector<double> joint_angles, const int speed, const int acc);
};

} // namespace rx1_motor

#endif // RX1_MOTOR_H
