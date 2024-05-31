#include "rx1_ik/rx1_ik.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rx1_ik");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    Rx1Ik rx1_ik_node(nh, priv_nh);

    try
    {
        rx1_ik_node.spin();
    }
    catch (std::runtime_error& ex)
    {
        ROS_FATAL_STREAM("[RX1_IK] Runtime error: " << ex.what());
        return 1;
    }

    return 0;
}

