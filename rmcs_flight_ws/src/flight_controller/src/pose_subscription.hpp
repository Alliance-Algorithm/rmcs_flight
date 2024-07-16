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

// dji_osdk
#include "common/dji_flight_control.hpp"
#include "common/dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include <djiosdk/dji_control.hpp>
#include <djiosdk/dji_telemetry.hpp>

#include <eigen3/Eigen/Eigen>

class PoseSubscription : public rclcpp::Node {
public:
    explicit PoseSubscription(int argc, char** argv)
        : Node("pose_subscription")
        , linuxEnvironment_(argc, argv)
    {
        initialize_djiosdk();

        pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rmcs_slam/position", 10,
            [this](const nav_msgs::msg::Odometry::UniquePtr& msg) { pose_subscription_callback(msg); });
        RCLCPP_INFO(get_logger(), "Subscribed to /rmcs_slam/position");

        monitoredTakeoff(vehicle_);

        initialize_telemetry();
        using namespace std::chrono_literals;
        main_process_timer_ = this->create_wall_timer(5ms,
            [this]() { main_process_timer_callback(); });
        RCLCPP_INFO(get_logger(), "Main process timer started.");
    };

    ~PoseSubscription()
    {
        release_telemtetry();
        vehicle_->releaseCtrlAuthority(1);

        RCLCPP_INFO(get_logger(), "\n---\n Release control authority \n---");
    };

private:
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> pose_subscription_;
    std::shared_ptr<rclcpp::TimerBase> main_process_timer_;
    Vehicle* vehicle_;
    LinuxSetup linuxEnvironment_;

    // mid360 datas
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mid360_euler_angles_ = Eigen::Vector3d::Zero();

    // PID parameters
    Eigen::Vector3d integral_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_position = Eigen::Vector3d(0.0, 0.0, -0.2);
    double kp = 20.0, ki = 0, kd = 0.1;

    // telemetry
    int responseTimeout = 1;
    // telemetry topics
    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type quaternion_;

private:
    /* --- initialization functions --- */

    void initialize_djiosdk()
    {
        // Initialize variables
        int functionTimeout = 1;

        // Setup OSDK.
        vehicle_ = linuxEnvironment_.getVehicle();
        if (vehicle_ == NULL) {
            RCLCPP_ERROR(get_logger(), "Vehicle not initialized, exiting.");
            rclcpp::shutdown();
        }

        // Obtain Control Authority
        ACK::ErrorCode result = vehicle_->obtainCtrlAuthority(functionTimeout);
        RCLCPP_INFO(get_logger(), "Obtain control authority result: %d", result.data);

        /*! Turn off rtk switch */
        ErrorCode::ErrorCodeType ret;
        ret = vehicle_->flightController->setRtkEnableSync(
            FlightController::RtkEnabled::RTK_DISABLE, 1);
        if (ret != ErrorCode::SysCommonErr::Success)
            RCLCPP_INFO(get_logger(), "Turn off rtk switch failed, ErrorCode is:%8lx", ret);
        else
            RCLCPP_INFO(get_logger(), "Turn off rtk switch successfully");

        RCLCPP_INFO(get_logger(), "Dji-osdk initializtion complete.");
    };

    bool initialize_telemetry()
    {
        vehicle_->subscribe->removePackage(0, responseTimeout);
        // Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle_->subscribe->verify(responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            return false;
        }

        // Subscribe to Quaternion at freq 100 Hz.
        int pkgIndex = 0;
        int freq = 200;
        DJI::OSDK::Telemetry::TopicName topicList200Hz[] = { DJI::OSDK::Telemetry::TOPIC_QUATERNION };
        int numTopic = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle_->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
        if (!(pkgStatus)) {
            return pkgStatus;
        }

        subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex, responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
            ACK::getErrorCodeMessage(subscribeStatus, __func__);
            // Cleanup before return
            vehicle_->subscribe->removePackage(pkgIndex, responseTimeout);
            return false;
        }

        // Wait for the data to start coming in.
        usleep(500 * 1000);
        return true;
        RCLCPP_INFO(get_logger(), "Telemetry initialization complete.");
    }

    void release_telemtetry()
    {
        vehicle_->subscribe->removePackage(0, responseTimeout);
    }

private:
    /* --- main process functions --- */

    void main_process_timer_callback()
    {
        receive_imu_data();
        // 当前误差
        Eigen::Vector3d error
            = target_position - position_;
        // 积分
        integral_error_ += error;
        // 微分
        Eigen::Vector3d derivative_error = error - last_error_;
        // 更新上一次误差
        last_error_ = error;

        // 计算PID输出
        Eigen::Vector3d control_input;
        control_input.x() = kp * error.x() + ki * integral_error_.x() + kd * derivative_error.x();
        control_input.y() = kp * error.y() + ki * integral_error_.y() + kd * derivative_error.y();
        control_input.z() = kp * error.z() + ki * integral_error_.z() + kd * derivative_error.z();

        control_input.x() = std::clamp(control_input.x(), -20., 20.);
        control_input.y() = std::clamp(control_input.y(), -20., 20.);
        control_input.z() = std::clamp(control_input.z(), -20., 20.);

        // RCLCPP_INFO(get_logger(), "position: %f, %f, %f", position_.x(), position_.y(), position_.z());
        // RCLCPP_INFO(get_logger(), "control_input: %f, %f, %f", control_input.x(), control_input.y(), control_input.z());

        vehicle_->control->velocityAndYawRateCtrl(
            control_input.x(),
            control_input.y(),
            control_input.z(),
            mid360_euler_angles_.z());
    }

    void pose_subscription_callback(const nav_msgs::msg::Odometry::UniquePtr& msg)
    {
        position_ = Eigen::Translation3d {
            msg->pose.pose.position.x,
            -msg->pose.pose.position.y,
            -msg->pose.pose.position.z
        }
                        .translation();

        mid360_euler_angles_ = toEulerAngle(Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z));
        mid360_euler_angles_ = Eigen::Vector3d(
            mid360_euler_angles_.y(),
            -mid360_euler_angles_.x(),
            mid360_euler_angles_.z());
    };

    void receive_imu_data()
    {
        quaternion_ = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();

        Eigen::Vector3d imu_euler_angles = toEulerAngle(
            Eigen::Quaterniond(
                quaternion_.q0,
                quaternion_.q1,
                quaternion_.q2,
                quaternion_.q3));

        std::cout << "Attitude Euler_angles (x,y,z) = (" << imu_euler_angles.x() / std::numbers::pi * 180
                  << ", " << imu_euler_angles.y() / std::numbers::pi * 180 << ", " << imu_euler_angles.z() / std::numbers::pi * 180 << ")\n";
        std::cout << "mid360 Euler_angles (x,y,z) = (" << mid360_euler_angles_.x()
                  << ", " << mid360_euler_angles_.y() << ", " << mid360_euler_angles_.z() << ")\n";
    }

private:
    /* --- utility functions --- */

    Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond& q)
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
};