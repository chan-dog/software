#pragma once

// #include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <random>
#include <rcl/event.h>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/detail/head_cmd__struct.hpp>

#include "hls.hpp"
#include "robot_interfaces/msg/head_cmd.hpp"
#include "robot_interfaces/msg/head_pose.hpp"
#include "serial_utils/serial_device.hpp"

namespace serial_device {
class Motor : public SerialDevice {
public:
    explicit Motor(const rclcpp::NodeOptions& opetions);

    ~Motor() override = default;

private:
    void receiveData() override final { };
    void getMotorParams();
    void initDevices();
    void sendData(const robot_interfaces::msg::HeadCmd::SharedPtr msg);

    // motors
    std::map<std::string, std::unique_ptr<feetech::HLS>> motors_;

    rclcpp::Subscription<robot_interfaces::msg::HeadCmd>::SharedPtr cmd_vel_sub_;
};
}