#include "rx1_motor/rx1_motor.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rx1_motor_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    rx1_motor::Rx1Motor rx1_motor_node(nh, priv_nh);

    try
    {
        rx1_motor_node.spin();
    }
    catch (std::runtime_error& ex)
    {
        ROS_FATAL_STREAM("[RX1_MOTOR] Runtime error: " << ex.what());
        return 1;
    }
    return 0;
}

