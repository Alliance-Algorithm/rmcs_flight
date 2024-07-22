#include "rmcs_flight_controller.hpp"

void RmcsFlightController::main_process_timer_callback()
{
    receive_subscription_data();

    if ((rc_mode_ == 0 && last_rc_mode_ == 0)) {
        if (debug_)
            RCLCPP_INFO(get_logger(), "Mauanl mode, waiting for rc mode change.");
        return;
    }
    if ((rc_mode_ == 1 && last_rc_mode_ == 0) || (rc_mode_ == 1 && last_rc_mode_ == 2)) {
        RCLCPP_INFO(get_logger(), "\n\n>>> Self-stabilization mode, start control.\n");

        // reset PID and target position
        target_position_ = mid360_position_;
        integral_error_ = Eigen::Vector3d::Zero();
        last_error_ = Eigen::Vector3d::Zero();

        // Obtain Control Authority
        ACK::ErrorCode result = vehicle_->obtainCtrlAuthority(1);

        RCLCPP_INFO(get_logger(), "Obtain control authority result: %d", result.data);
        if (result.data == 2)
            RCLCPP_INFO(get_logger(), "\n--- current control authority is OSDK.");
        else
            RCLCPP_ERROR(get_logger(), "\n--- Obtain control authority failed!");
    }
    if ((rc_mode_ == 0 && last_rc_mode_ == 1)) {
        RCLCPP_INFO(get_logger(), "\n\n>>> Manual mode, release control authority.\n");
        ACK::ErrorCode result = vehicle_->releaseCtrlAuthority(1);
        RCLCPP_INFO(get_logger(), "Release control authority result: %d", result.data);
        if (result.data == 0)
            RCLCPP_INFO(get_logger(), "\n--- current control authority is RC.");
        else
            RCLCPP_ERROR(get_logger(), "\n--- Release control authority failed!");
        return;
    }


    // // 当前误差
    // Eigen::Vector3d error
    //     = target_position_ - mid360_position_;
    // // 积分
    // integral_error_ += error;
    // integral_error_.x() = std::clamp(integral_error_.x(), -10., 10.);
    // integral_error_.y() = std::clamp(integral_error_.y(), -10., 10.);
    // integral_error_.z() = std::clamp(integral_error_.z(), -10., 10.);

    // // 微分
    // Eigen::Vector3d derivative_error = error - last_error_;
    // // 更新上一次误差
    // last_error_ = error;

    // // 计算PID输出
    // Eigen::Vector3d control_input;
    // control_input.x() = kp_ * error.x() + ki_ * integral_error_.x() + kd_ * derivative_error.x();
    // control_input.y() = kp_ * error.y() + ki_ * integral_error_.y() + kd_ * derivative_error.y();
    // control_input.z() = kp_ * error.z() + ki_ * integral_error_.z() + kd_ * derivative_error.z();

    // control_input.x() = std::clamp(control_input.x(), -5., 5.);
    // control_input.y() = std::clamp(control_input.y(), -5., 5.);
    // control_input.z() = std::clamp(control_input.z(), -5., 5.);

    // if (debug_)
    //     RCLCPP_INFO(get_logger(), "control_input: %f,%f,%f", control_input.x(), control_input.y(), control_input.z());

    // vehicle_->control->velocityAndYawRateCtrl(
    //     control_input.x(),
    //     control_input.y(),
    //     control_input.z(),
    //     0);

    angularAndYawRateCtrl(0, 10, 0, 0);
}
