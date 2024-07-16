#include "pose_subscription.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSubscription>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// int main(int argc, char** argv)
// {
//     // Initialize variables
//     int functionTimeout = 1;
//     int count = 0;
//     int controlFreqInHz = 50; // Hz
//     int cycleTimeInMs = 1000 / controlFreqInHz;

//     // Setup OSDK.
//     LinuxSetup linuxEnvironment(argc, argv);
//     auto vehicle = linuxEnvironment.getVehicle();
//     if (vehicle == NULL) {
//         std::cout << "Vehicle not initialized, exiting.\n";
//         return -1;
//     }

//     // Obtain Control Authority
//     vehicle->obtainCtrlAuthority(functionTimeout);

//     /*! Turn off rtk switch */
//     ErrorCode::ErrorCodeType ret;
//     ret = vehicle->flightController->setRtkEnableSync(
//         FlightController::RtkEnabled::RTK_DISABLE, 1);
//     if (ret != ErrorCode::SysCommonErr::Success) {
//         DSTATUS("Turn off rtk switch failed, ErrorCode is:%8x", ret);
//     } else {
//         DSTATUS("Turn off rtk switch successfully");
//     }

//     /*!  Take off */
//     monitoredTakeoff(vehicle);

//     // while (count < 250) {
//     //     vehicle->control->velocityAndYawRateCtrl(0, DJI::OSDK::Control::HORIZONTAL_VELOCITY, 0.1, 0);
//     //     usleep(cycleTimeInMs * 1000);
//     //     count++;
//     // }

//     // count = 0;

//     // while (count < 250) {
//     //     vehicle->control->velocityAndYawRateCtrl(DJI::OSDK::Control::HORIZONTAL_VELOCITY, 0, 0, 0);
//     //     usleep(cycleTimeInMs * 1000);
//     //     count++;
//     // }
//     // count = 0;
//     // while (count < 250) {
//     //     vehicle->control->velocityAndYawRateCtrl(0, -DJI::OSDK::Control::HORIZONTAL_VELOCITY, 0, 0);
//     //     usleep(cycleTimeInMs * 1000);
//     //     count++;
//     // }
//     // count = 0;
//     // while (count < 250) {
//     //     vehicle->control->velocityAndYawRateCtrl(-DJI::OSDK::Control::HORIZONTAL_VELOCITY, 0, 0, 0);
//     //     usleep(cycleTimeInMs * 1000);
//     //     count++;
//     // }

//     // // moveByPositionOffset(vehicle, -10, 0, 0, 60);

//     // /*! landing */
//     // monitoredLanding(vehicle);

//     return 0;
// }
