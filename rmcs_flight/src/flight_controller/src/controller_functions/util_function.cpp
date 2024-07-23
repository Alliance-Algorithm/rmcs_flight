#include "rmcs_flight_controller.hpp"

namespace rmcs_flight {
Eigen::Vector3d RmcsFlightController::to_euler_angle(const Eigen::Quaterniond& q)
{
    double roll, pitch, yaw;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(roll, pitch, yaw);
}

void RmcsFlightController::angular_and_yaw_rate_ctrl(float roll, float pitch, float yaw_rate, float z_velo)
{
    uint8_t ctrl_flag = static_cast<uint8_t>(DJI::OSDK::Control::VERTICAL_VELOCITY)
        | static_cast<uint8_t>(DJI::OSDK::Control::HORIZONTAL_ANGLE)
        | static_cast<uint8_t>(DJI::OSDK::Control::YAW_RATE)
        | static_cast<uint8_t>(DJI::OSDK::Control::HORIZONTAL_BODY);
    //! @note 10 is the flag value of this mode
    DJI::OSDK::Control::CtrlData data(ctrl_flag, roll, pitch, z_velo, yaw_rate);
    vehicle_->control->flightCtrl(data);
}

bool RmcsFlightController::if_self_stable()
{
    if ((last_rc_mode_ == rc_mode_) && (rc_mode_ != 1)) {
        if (debug_)
            RCLCPP_INFO(get_logger(), "Mauanl mode, waiting for rc mode change.");

        return false;

    } else if ((last_rc_mode_ == 1) && (rc_mode_ != 1)) {
        RCLCPP_INFO(get_logger(), "\n\n>>> Manual mode, release control authority.\n");

        ACK::ErrorCode result = vehicle_->releaseCtrlAuthority(1);
        RCLCPP_INFO(get_logger(), "Release control authority result: %d", result.data);

        if (result.data == 0)
            RCLCPP_INFO(get_logger(), "\n--- current control authority is RC.");
        else
            RCLCPP_ERROR(get_logger(), "\n--- Release control authority failed!");

        return false;

    } else if ((last_rc_mode_ != 1) && (rc_mode_ == 1)) {
        RCLCPP_INFO(get_logger(), "\n\n>>> Self-stabilization mode, start control.\n");

        // reset PID and target position
        pid_controller_.reset_pid(mid360_position_);

        // Obtain Control Authority
        ACK::ErrorCode result = vehicle_->obtainCtrlAuthority(1);
        RCLCPP_INFO(get_logger(), "Obtain control authority result: %d", result.data);

        if (result.data == 2)
            RCLCPP_INFO(get_logger(), "\n--- current control authority is OSDK.");
        else
            RCLCPP_ERROR(get_logger(), "\n--- Obtain control authority failed!");

        return true;
    } else
        return true;
}
}