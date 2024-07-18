#include "common/dji_flight_control.hpp"

#include "rmcs_flight_controller.hpp"

void RmcsFlightController::initialization()
{
    // load parameters
    load_parameters();

    // initialize dji osdk
    initialize_djiosdk();

    // subscribe to /rmcs_slam/position
    pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        mid360_data_topic_, 10,
        [this](const nav_msgs::msg::Odometry::UniquePtr& msg) { pose_subscription_callback(msg); });

    RCLCPP_INFO(get_logger(), "Subscribed to %s", mid360_data_topic_.c_str());

    // wait for takeoff if not
    if (debug_)
        monitoredTakeoff(vehicle_);

    // initialize telemetry
    initialize_telemetry();

    // start main process timer
    main_process_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000 / control_frequency_hz_)),
        [this]() { main_process_timer_callback(); });

    RCLCPP_INFO(get_logger(), "\n[√] Main process timer started.");
};

void RmcsFlightController::initialize_djiosdk()
{
    // Initialize variables
    int functionTimeout = 1;

    // Setup OSDK.
    vehicle_ = linuxEnvironment_->getVehicle();
    if (vehicle_ == NULL) {
        RCLCPP_ERROR(get_logger(), "\n[x] Vehicle not initialized, exiting.");
        rclcpp::shutdown();
    }

    // Obtain Control Authority
    ACK::ErrorCode result = vehicle_->obtainCtrlAuthority(functionTimeout);
    RCLCPP_INFO(get_logger(), "Obtain control authority result: %d", result.data);

    // Turn off rtk switch
    ErrorCode::ErrorCodeType ret;
    ret = vehicle_->flightController->setRtkEnableSync(
        FlightController::RtkEnabled::RTK_DISABLE, 1);
    if (ret != ErrorCode::SysCommonErr::Success)
        RCLCPP_INFO(get_logger(), "Turn off rtk switch failed, ErrorCode is:%8lx", ret);
    else
        RCLCPP_INFO(get_logger(), "Turn off rtk switch successfully");

    RCLCPP_INFO(get_logger(), "\n[√] Dji-osdk initializtion complete.");
};

bool RmcsFlightController::initialize_telemetry()
{
    vehicle_->subscribe->removePackage(0, responseTimeout_);
    vehicle_->subscribe->removePackage(1, responseTimeout_);
    // Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle_->subscribe->verify(responseTimeout_);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
    }

    // Subscribe to Quaternion at freq 200 Hz.
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

    subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex, responseTimeout_);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle_->subscribe->removePackage(pkgIndex, responseTimeout_);
        return false;
    }

    // Subscribe to RC Channel at freq 50 Hz
    pkgIndex = 1;
    freq = 50;
    DJI::OSDK::Telemetry::TopicName topicList50Hz[] = { DJI::OSDK::Telemetry::TOPIC_RC };
    numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    enableTimestamp = false;

    pkgStatus = vehicle_->subscribe->initPackageFromTopicList(
        pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus)) {
        return pkgStatus;
    }
    subscribeStatus = vehicle_->subscribe->startPackage(pkgIndex, responseTimeout_);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup before return
        vehicle_->subscribe->removePackage(pkgIndex, responseTimeout_);
        return false;
    }

    // Wait for the data to start coming in.
    usleep(500 * 1000);
    RCLCPP_INFO(get_logger(), "\n[√] Telemetry initialization complete.");
    return true;
}

void RmcsFlightController::load_parameters()
{
    debug_ = get_parameter_or<bool>("debug", false);

    kp_ = get_parameter_or<double>("pid.kp", 0.5);
    ki_ = get_parameter_or<double>("pid.ki", 0.0);
    kd_ = get_parameter_or<double>("pid.kd", 0.1);

    control_frequency_hz_ = get_parameter_or<int>("control_frequency", 50);
    mid360_data_topic_ = get_parameter_or<std::string>("mid360_data_topic", "/rmcs_slam/position");

    auto dji_config_path = get_parameter_or<std::string>("dji.config_path", "");

    RCLCPP_INFO(get_logger(), "Loaded parameters: kp=%f, ki=%f, kd=%f, dji_config_path=%s",
        kp_, ki_, kd_, dji_config_path.data());

    char* argv[] = {
        nullptr,
        dji_config_path.data(),
    };
    linuxEnvironment_ = new LinuxSetup(2, argv);
}

void RmcsFlightController::release_telemtetry()
{
    vehicle_->subscribe->removePackage(0, responseTimeout_);
    vehicle_->subscribe->removePackage(1, responseTimeout_);
    std::cout << "\n---[√] Release telemetry complete.\n"
              << std::endl;
}