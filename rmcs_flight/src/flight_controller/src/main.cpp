#include "rmcs_flight_controller.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rmcs_flight::RmcsFlightController>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}