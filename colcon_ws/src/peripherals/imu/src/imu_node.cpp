#include "imu/imu_node.hpp"
#include <cmath>
#include <rclcpp/qos.hpp>

namespace serial_device
{

Imu::Imu(const rclcpp::NodeOptions& options)
    : SerialDevice("Imu_node", options, true),
      current_state_(acc) 
{
    RCLCPP_INFO(get_logger(), "IMU node started (sequence: acc→gyro→angle→quat)");

    imu_msg_.header.frame_id = "imu_link"; 
    auto qos = rclcpp::SensorDataQoS().keep_last(10); 
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", qos);

    std::fill(imu_msg_.linear_acceleration_covariance.begin(), 
              imu_msg_.linear_acceleration_covariance.end(), 0.01);
    std::fill(imu_msg_.angular_velocity_covariance.begin(), 
              imu_msg_.angular_velocity_covariance.end(), 0.01);
    std::fill(imu_msg_.orientation_covariance.begin(), 
              imu_msg_.orientation_covariance.end(), 0.01);

    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(800),
        std::bind(&Imu::resetState, this)
    );
}

uint8_t Imu::calculateCheckSum(const ImuDataPacket& packet)
{
    uint16_t sum = 0;
    sum += packet.frame;
    sum += packet.type;
    sum += packet.data.x_u8[0]; sum += packet.data.x_u8[1];
    sum += packet.data.y_u8[0]; sum += packet.data.y_u8[1];
    sum += packet.data.z_u8[0]; sum += packet.data.z_u8[1];
    sum += packet.data.keep_u8[0]; sum += packet.data.keep_u8[1];
    return static_cast<uint8_t>(sum & 0xFF);
}

void Imu::handleReceivedData(const std::vector<uint8_t>& data)
{
    if (data.empty()) return;
    recv_buffer_.insert(recv_buffer_.end(), data.begin(), data.end());
    parseBuffer();
}

void Imu::parseBuffer()
{
    const size_t MAX_BUFFER = FRAME_LEN * 20;
    if (recv_buffer_.size() > MAX_BUFFER)
    {
        RCLCPP_WARN(get_logger(), "Buffer overflow! Truncated (size: %zu→%zu)",
                   recv_buffer_.size(), MAX_BUFFER);
        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.end() - MAX_BUFFER);
    }

    while (recv_buffer_.size() >= FRAME_LEN)
    {
        size_t header_idx = -1;
        for (size_t i = 0; i <= recv_buffer_.size() - FRAME_LEN; ++i)
        {
            if (recv_buffer_[i] == FRAME_HEADER)
            {
                header_idx = i;
                break;
            }
        }

        if (header_idx == static_cast<size_t>(-1))
        {
            recv_buffer_.pop_front();
            continue;
        }

        ImuDataPacket packet;
        std::memcpy(&packet, &recv_buffer_[header_idx], FRAME_LEN);

        if (calculateCheckSum(packet) != packet.sum_crc)
        {
            RCLCPP_WARN(get_logger(), "Checksum mismatch! Recv=0x%02X, Calc=0x%02X",
                       packet.sum_crc, calculateCheckSum(packet));
            recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + header_idx + 1);
            continue;
        }

        processPacket(packet);

        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + header_idx + FRAME_LEN);
    }
}

void Imu::processPacket(const ImuDataPacket& packet)
{
    if (packet.type != acc && packet.type != ang_vel && 
        packet.type != angle && packet.type != quat)
    {
        RCLCPP_DEBUG(get_logger(), "Ignored irrelevant frame (TYPE=0x%02X)", packet.type);
        return;
    }

    if (packet.type != current_state_)
    {
        RCLCPP_WARN(get_logger(), "Sequence broken! Expected %s(0x%02X), got 0x%02X",
                   (current_state_==acc?"acc":
                    current_state_==ang_vel?"gyro":
                    current_state_==angle?"angle":"quat"),
                   current_state_, packet.type);
        current_state_ = acc;  
        return;
    }

    imu_msg_.header.stamp = now(); 
    switch (packet.type)
    {
        case acc: 
            imu_msg_.linear_acceleration.x = static_cast<float>(packet.data.x) * ACC_SCALE * 9.81f;
            imu_msg_.linear_acceleration.y = static_cast<float>(packet.data.y) * ACC_SCALE * 9.81f;
            imu_msg_.linear_acceleration.z = static_cast<float>(packet.data.z) * ACC_SCALE * 9.81f;
            RCLCPP_DEBUG(get_logger(), "Step1/4: Acc received (x=%.2f, y=%.2f, z=%.2f)",
                       imu_msg_.linear_acceleration.x, imu_msg_.linear_acceleration.y, imu_msg_.linear_acceleration.z);
            current_state_ = ang_vel;  
            break;

        case ang_vel: 
            imu_msg_.angular_velocity.x = static_cast<float>(packet.data.x) * GYRO_SCALE * M_PI / 180.0f;
            imu_msg_.angular_velocity.y = static_cast<float>(packet.data.y) * GYRO_SCALE * M_PI / 180.0f;
            imu_msg_.angular_velocity.z = static_cast<float>(packet.data.z) * GYRO_SCALE * M_PI / 180.0f;
            RCLCPP_DEBUG(get_logger(), "Step2/4: Gyro received (x=%.2f, y=%.2f, z=%.2f)",
                       imu_msg_.angular_velocity.x, imu_msg_.angular_velocity.y, imu_msg_.angular_velocity.z);
            current_state_ = angle; 
            break;

        case angle:  
            RCLCPP_DEBUG(get_logger(), "Step3/4: Angle received (x=%.2f°, y=%.2f°, z=%.2f°)",
                       static_cast<float>(packet.data.x) * ANGLE_SCALE * M_PI / 180.0f,
                       static_cast<float>(packet.data.y) * ANGLE_SCALE * M_PI / 180.0f,
                       static_cast<float>(packet.data.z) * ANGLE_SCALE * M_PI / 180.0f);
            current_state_ = quat;  
            break;

        case quat:  
            imu_msg_.orientation.x = static_cast<float>(packet.data.x) * QUAT_SCALE;
            imu_msg_.orientation.y = static_cast<float>(packet.data.y) * QUAT_SCALE;
            imu_msg_.orientation.z = static_cast<float>(packet.data.z) * QUAT_SCALE;
            imu_msg_.orientation.w = static_cast<float>(packet.data.keep) * QUAT_SCALE;
            

            imu_pub_->publish(imu_msg_);
            RCLCPP_INFO(get_logger(), "Step4/4: Complete data published (quat: x=%.3f y=%.3f z=%.3f w=%.3f)",
                       imu_msg_.orientation.x, imu_msg_.orientation.y, imu_msg_.orientation.z, imu_msg_.orientation.w);
            
            current_state_ = acc; 
            break;
    }

    timeout_timer_->reset();
}

void Imu::resetState()
{
    if (current_state_ != acc)
    {
        RCLCPP_WARN(get_logger(), "Timeout! Stuck at step %d/%d (%s). Resetting...",
                   (current_state_==acc?1:(current_state_==ang_vel?2:(current_state_==angle?3:4))),
                   4,
                   (current_state_==acc?"acc":(current_state_==ang_vel?"gyro":(current_state_==angle?"angle":"quat"))));
        current_state_ = acc;
    }
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_device::Imu)