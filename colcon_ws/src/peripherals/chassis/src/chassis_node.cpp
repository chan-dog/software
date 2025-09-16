#include <rclcpp/node_options.hpp>
#include "chassis/chassis_node.hpp"
#include "serial_utils/package.hpp"
#include <sstream>  
#include <iomanip>  

namespace serial_device {

uint8_t Chassis::getCheckSum(const std::vector<uint8_t>& _data)
{
    uint16_t sum = 0; 
    uint8_t len = static_cast<uint8_t>(_data.size());

    for (int i = 0; i < len; i++)
    {
        sum += _data[i];
    };
    return static_cast<uint8_t>(sum & 0xFF);  
}

Chassis::Chassis(const rclcpp::NodeOptions& options)
    : SerialDevice("chassis_node", options, true)
{
    RCLCPP_INFO(get_logger(), "Chassis node started.");
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_chassis", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            try {
                // 将 float * 1000 -> int16
                int16_t x_val = static_cast<int16_t>(msg->linear.x * 1000.0f);
                int16_t z_val = static_cast<int16_t>(msg->angular.z * 1000.0f);

                // 构造数据包
                CmdVelPacket packet;
                packet.data.x_high = static_cast<uint8_t>((x_val >> 8) & 0xFF);
                packet.data.x_low = static_cast<uint8_t>(x_val & 0xFF);
                packet.data.z_high = static_cast<uint8_t>((z_val >> 8) & 0xFF);
                packet.data.z_low = static_cast<uint8_t>(z_val & 0xFF);
                packet.check_sum = getCheckSum(toVector(packet.data));

                std::vector<uint8_t> data = toVector(packet);
                
                serial_driver_->port()->send(data);

                // std::ostringstream oss;
                // oss << "Sent cmd_vel data: [ ";
                // for (size_t i = 0; i < data.size(); i++) {
                //     oss << static_cast<int>(data[i]) << " ";
                // }
                // oss << "]";
                // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

            } catch (const std::exception& ex) {
                RCLCPP_ERROR(get_logger(), "Error while sending cmd_vel data: %s", ex.what());
                reopenPort();
            }
        });
}

void Chassis::handleReceivedData(const std::vector<uint8_t>& data)
{
    if (data.empty() || data.size() < sizeof(StatusPacket)) {
        RCLCPP_WARN(get_logger(), "Received data invalid: empty or too short (size: %zu, required: %zu)",
                    data.size(), sizeof(StatusPacket));
        return;
    }

    StatusPacket packet;
    if (data[0] != packet.header || data[1] != packet.data_len) {
        RCLCPP_ERROR(get_logger(), "Invalid packet header/length: header=0x%02X (expected=0x%02X), data_len=%hhu (expected=%hhu)",
                    static_cast<int>(data[0]), static_cast<int>(packet.header),
                    data[1], packet.data_len);
        return;
    }

    // std::stringstream hex_ss;
    // hex_ss << "Received data (HEX): ";
    // for (size_t i = 0; i < data.size(); ++i) { 
    //     hex_ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
    //            << static_cast<int>(data[i]) << " ";  
    // }
    // RCLCPP_INFO(get_logger(), "%s", hex_ss.str().c_str());

    memcpy(packet.data.miles_u8, &data[2], 4);  
    memcpy(packet.data.voltage_u8, &data[6], 4); 
    packet.check_sum = data[sizeof(StatusPacket) - 1];              

    uint8_t calculated_checksum = 0;
    calculated_checksum = getCheckSum(toVector(packet.data));
    if (calculated_checksum != packet.check_sum) {
        RCLCPP_ERROR(get_logger(), "Checksum mismatch: received=0x%02X, calculated=0x%02X",
                    static_cast<int>(packet.check_sum), static_cast<int>(calculated_checksum));
        return;
    }

    miles = packet.data.miles;
    voltage = packet.data.voltage;
    RCLCPP_INFO(get_logger(), "Status data parsed successfully: Miles=%.2f, Voltage=%.2fV",
                miles, voltage);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(serial_device::Chassis)