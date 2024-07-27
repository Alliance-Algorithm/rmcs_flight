#include "rmcs_flight_controller.hpp"

namespace rmcs_flight {
void RmcsFlightController::pose_subscription_callback(const nav_msgs::msg::Odometry::UniquePtr& msg)
{
    mid360_position_ = Eigen::Vector3d {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        -msg->pose.pose.position.z
    };
    mid360_quaternion_ = Eigen::Quaterniond{
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    };
    

};

void RmcsFlightController::receive_subscription_data()
{
    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type imu_raw_quaternion;
    imu_raw_quaternion = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();

    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC>::type rc_data;
    rc_data = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC>();

    last_rc_mode_ = rc_mode_;
    if (rc_data.mode > 0) {
        rc_mode_ = 2;
    } else if (rc_data.mode == 0) {
        rc_mode_ = 1;
    } else {
        rc_mode_ = 0;
    }

    // RCLCPP_INFO(get_logger(), "mid_360: %f,%f,%f", mid360_position_.x(), mid360_position_.y(), mid360_position_.z());
}
}