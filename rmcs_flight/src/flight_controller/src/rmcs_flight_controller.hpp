#pragma once

// std
#include <memory>
#include <unistd.h>

// ros2
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/transform_broadcaster.h>

// dji_osdk
#include "common/dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include <djiosdk/dji_control.hpp>
#include <djiosdk/dji_telemetry.hpp>

// eigen
#include <eigen3/Eigen/Eigen>

// pid
#include "double_loop_pid/pid_controller.hpp"

namespace rmcs_flight {
class RmcsFlightController : public rclcpp::Node {
public:
    explicit RmcsFlightController(int argc, char** argv);

    ~RmcsFlightController();

private:
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> pose_subscription_;
    std::shared_ptr<rclcpp::TimerBase> main_process_timer_;
    Vehicle* vehicle_;
    LinuxSetup* linuxEnvironment_;

    // mid360 datas
    std::string mid360_data_topic_;
    Eigen::Vector3d mid360_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mid360_euler_angles_ = Eigen::Vector3d::Zero();

    // PID parameters
    double kp_pos_, ki_pos_, kd_pos_;
    double kp_velo_, ki_velo_, kd_velo_;
    DoubleLoopPIDController pid_controller_;

    // telemetry
    int responseTimeout_
        = 1;
    Eigen::Vector3d imu_euler_angles_;
    int rc_mode_ = -1;
    int last_rc_mode_ = -1;
    float altitude_;

    // other parameters
    int control_frequency_hz_;
    int debug_;

private:
    /* --- main process functions --- */

    void main_process_timer_callback();

    /* --- data callback functions --- */

    void pose_subscription_callback(const nav_msgs::msg::Odometry::UniquePtr& msg);
    void receive_subscription_data();

    /* --- utility functions --- */

    Eigen::Vector3d to_euler_angle(const Eigen::Quaterniond& q);
    void angular_and_yaw_rate_ctrl(float roll, float pitch, float yaw_rate, float z_velo);
    bool if_self_stable();

    /* --- initialization functions --- */

    void initialization();
    void initialize_djiosdk();
    bool initialize_telemetry();
    void load_parameters();
    void release_telemtetry();
};
}