#include "rx1_motor/rx1_motor.hpp"

namespace rx1_motor
{
Rx1Motor::Rx1Motor(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
    : nh_(nh),
      priv_nh_(priv_nh)
{
    servo_port_ = "/dev/ttyUSB-arduino4.3";

    if (!sts_servo_.begin(1000000, servo_port_.c_str()))
    {
        ROS_ERROR("[RX1_MOTOR] Failed initialize servo!");
    }
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
    }
}

void Rx1Motor::update()
{
}

void Rx1Motor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Access the joint state information
    std::vector<std::string> joint_names = msg->name;
    std::vector<double> joint_positions = msg->position;

    // Process the joint state information
    motorCommand(joint_positions, 0, 40);

    // Print joint names and positions
    /*
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint Name: %s, Position: %f", joint_names[i].c_str(), joint_positions[i]);
    }
    */
}

void Rx1Motor::motorCommand(const std::vector<double> joint_angles, const int speed=100, const int acc= 20)
{
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

    sts_servo_.SyncWritePosEx(ids, 7, pos, speeds, accs); 
}

} // namespace hrcmodel4_motor
