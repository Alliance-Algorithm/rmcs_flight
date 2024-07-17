#include "rmcs_flight_controller.hpp"

RmcsFlightController::RmcsFlightController(int argc, char** argv)
    : Node("rmcs_flight_controller", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    initialization();
    RCLCPP_INFO(get_logger(), "Initialization complete.");
};

RmcsFlightController::~RmcsFlightController()
{
    /* --- release ctrl authority --- */
    release_telemtetry();
    vehicle_->releaseCtrlAuthority(1);
    RCLCPP_INFO(get_logger(), "\n\n--- Successfully release control authority. \n--- Exited. \n");
};
