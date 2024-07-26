#pragma once

#include <eigen3/Eigen/Eigen>

namespace rmcs_flight::pid_parameters {
struct PositonPID {
    double Kp;
    double Ki;
    double Kd;
    Eigen::Vector3d integral_error;
    Eigen::Vector3d last_error;
    Eigen::Vector3d current_error;
};
struct VelocityPID {
    double Kp;
    double Ki;
    double Kd;
    Eigen::Vector3d integral_error;
    Eigen::Vector3d last_error;
    Eigen::Vector3d current_error;
};
}