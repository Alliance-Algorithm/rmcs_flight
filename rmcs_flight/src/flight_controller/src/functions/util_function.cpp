#include "rmcs_flight_controller.hpp"

Eigen::Vector3d RmcsFlightController::toEulerAngle(const Eigen::Quaterniond& q)
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

void RmcsFlightController::angularAndYawRateCtrl(float roll,float pitch,float yaw_rate,float z_velo)
{
    uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_VELOCITY | DJI::OSDK::Control::HORIZONTAL_ANGLE | DJI::OSDK::Control::YAW_RATE |
                    DJI::OSDK::Control::HORIZONTAL_BODY);
    //! @note 10 is the flag value of this mode
    DJI::OSDK::Control::CtrlData data(ctrl_flag, roll, pitch, z_velo, yaw_rate);
    vehicle_->control->flightCtrl(data);
}
