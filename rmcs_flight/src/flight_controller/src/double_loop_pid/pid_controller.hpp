#pragma once

#include "pid_parameters.hpp"
#include <eigen3/Eigen/Eigen>

namespace rmcs_flight {

class DoubleLoopPIDController {
public:
    DoubleLoopPIDController(double kp_pos, double ki_pos, double kd_pos,
        double kp_velo, double ki_velo, double kd_velo)
    {
        pos_para = {
            kp_pos,
            ki_pos,
            kd_pos,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()
        };
        velo_para = {
            kp_velo,
            ki_velo,
            kd_velo,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()
        };
    };

    Eigen::Vector3d calculate_control_input(Eigen::Vector3d current_position)
    {
        // outer pid (position)
        pos_para.current_error = target_position_ - current_position;

        Eigen::Vector3d velocity_input = pos_para.Kp * pos_para.current_error + pos_para.Ki * pos_para.integral_error + pos_para.Kd * (pos_para.current_error - pos_para.last_error);

        pos_para.integral_error += pos_para.current_error;
        pos_para.last_error = pos_para.current_error;

        // inner pid (velocity)
        Eigen::Vector3d current_velocity = current_position - last_position_;
        velo_para.current_error = velocity_input - current_velocity;

        Eigen::Vector3d control_input = velo_para.Kp * velo_para.current_error + velo_para.Ki * velo_para.integral_error + velo_para.Kd * (velo_para.current_error - velo_para.last_error);

        velo_para.integral_error += velo_para.current_error;
        velo_para.last_error = velo_para.current_error;

        last_position_ = current_position;

        return control_input;
    }

    void reset_pid(Eigen::Vector3d target_position)
    {
        target_position_ = target_position;

        pos_para.current_error = Eigen::Vector3d::Zero();
        pos_para.integral_error = Eigen::Vector3d::Zero();
        pos_para.last_error = Eigen::Vector3d::Zero();
        velo_para.current_error = Eigen::Vector3d::Zero();
        velo_para.integral_error = Eigen::Vector3d::Zero();
        velo_para.last_error = Eigen::Vector3d::Zero();
    }

private:
    pid_parameters::PositonPID pos_para;
    pid_parameters::VelocityPID velo_para;

    Eigen::Vector3d target_position_, last_position_;
};
}