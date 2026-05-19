#include "rx1_motor/rx1_motor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rx1_motor::Rx1Motor>();

    try {
        node->spin();
    } catch (const std::runtime_error & ex) {
        RCLCPP_FATAL(node->get_logger(), "[RX1_MOTOR] Runtime error: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
