#include "rmcs_flight_controller.hpp"

void RmcsFlightController::pose_subscription_callback(const nav_msgs::msg::Odometry::UniquePtr& msg)
{
    mid360_position_ = Eigen::Translation3d {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
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

void RmcsFlightController::receive_subscription_data()
{
    last_rc_mode_ = rc_mode_;

    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_QUATERNION>::type imu_raw_quaternion;
    imu_raw_quaternion = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();

    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_RC>::type rc_data;
    rc_data = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC>();

    DJI::OSDK::Telemetry::TypeMap<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_BAROMETER>::type altitude;
    altitude = vehicle_->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_BAROMETER>();
    altitude_ = static_cast<float>(altitude);

    if (rc_data.mode > 0) {
        rc_mode_ = 2;
    } else if (rc_data.mode == 0) {
        rc_mode_ = 1;
    } else {
        rc_mode_ = 0;
    }

    imu_euler_angles_ = toEulerAngle(
        Eigen::Quaterniond(
            imu_raw_quaternion.q0,
            imu_raw_quaternion.q1,
            imu_raw_quaternion.q2,
            imu_raw_quaternion.q3));


    // std::cout << "Attitude Euler_angles (x,y,z) = (" << imu_euler_angles_.x() / std::numbers::pi * 180
    //           << ", " << imu_euler_angles_.y() / std::numbers::pi * 180 << ", " << imu_euler_angles_.z() / std::numbers::pi * 180 << ")\n";

    // std::cout << "mid360 Euler_angles (x,y,z) = (" << mid360_euler_angles_.x()
    //           << ", " << mid360_euler_angles_.y() << ", " << mid360_euler_angles_.z() << ")\n";
    RCLCPP_INFO(get_logger(),"mid_360: %f,%f,%f", mid360_position_.x(), mid360_position_.y(),mid360_position_.z());
}
