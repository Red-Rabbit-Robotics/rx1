#include "rx1_ik/rx1_ik.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Rx1Ik>();

    try {
        node->spin();
    } catch (const std::runtime_error & ex) {
        RCLCPP_FATAL(node->get_logger(), "[RX1_IK] Runtime error: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
