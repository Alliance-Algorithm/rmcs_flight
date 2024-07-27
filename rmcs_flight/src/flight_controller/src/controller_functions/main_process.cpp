#include "rmcs_flight_controller.hpp"

namespace rmcs_flight {
void RmcsFlightController::main_process_timer_callback()
{
    receive_subscription_data();

    if (!if_self_stable())
        return;

    Eigen::Vector3d control_input = pid_controller_.calculate_control_input(mid360_position_);
    Eigen::Vector3d control_output = Eigen::Vector3d {
        -control_input.x(),
        -control_input.y(),
        control_input.z()
    };

    std::cout << "control : " << control_output.x() << "," <<  control_output.y() << std::endl;

    control_output = to_drone_coordinate(control_output, mid360_quaternion_);

    angular_and_yaw_rate_ctrl(
        control_output.x(),
        control_output.y(),
        0,
        0);
}
}