#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node_options.hpp>

#include "serial_utils/serial_device.hpp"
#include <vector>

namespace serial_device {

struct CmdVelData
{
    uint8_t x_high;
    uint8_t x_low;
    uint16_t keep = 0;
    uint8_t z_high;
    uint8_t z_low;
} __attribute__((packed));

struct StatusData
{//按小端排序
union {
    float miles;
    uint8_t miles_u8[4];
};
union {
    float voltage;
    uint8_t voltage_u8[4];
};
} __attribute__((packed));

struct CmdVelPacket {
    uint8_t header = 0x7B;
    uint8_t data_len = sizeof(CmdVelData);
    CmdVelData data;
    uint8_t check_sum ;
} __attribute__((packed));

struct StatusPacket {
    uint8_t header = 0x7B;
    uint8_t data_len = sizeof(StatusData);
    StatusData data;
    uint8_t check_sum ;
} __attribute__((packed));


class Chassis : public SerialDevice {
public:
    explicit Chassis(const rclcpp::NodeOptions& options);

    ~Chassis() override = default;

private:
    float miles;
    float voltage;
    uint8_t getCheckSum(const std::vector<uint8_t>& _data);
    void handleReceivedData(const std::vector<uint8_t>& _data) override;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};
}