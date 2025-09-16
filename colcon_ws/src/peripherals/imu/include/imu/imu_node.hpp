#ifndef __IMU__HPP__
#define __IMU__HPP__

#include <cstdint>
#include <deque>
#include <vector>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "serial_utils/serial_device.hpp"

namespace serial_device
{

enum ImuDataType
{
    acc = 0x51,  
    ang_vel = 0x52, 
    angle = 0x53,   
    quat = 0x59  
};

//小端序存储
struct ImuData
{
    union { short x; uint8_t x_u8[2]; };  
    union { short y; uint8_t y_u8[2]; }; 
    union { short z; uint8_t z_u8[2]; };  
    union { short keep; uint8_t keep_u8[2]; };
} __attribute__((packed));


struct ImuDataPacket
{
    uint8_t frame = 0x55;  
    uint8_t type;         
    ImuData data;      
    uint8_t sum_crc;      
} __attribute__((packed));

class Imu : public SerialDevice
{
public: 
    explicit Imu(const rclcpp::NodeOptions& options);
    ~Imu() override = default;

private:

    uint8_t calculateCheckSum(const ImuDataPacket& packet);  
    void handleReceivedData(const std::vector<uint8_t>& data) override; 
    void parseBuffer(); 
    void processPacket(const ImuDataPacket& packet); 
    void resetState();   


    std::deque<uint8_t> recv_buffer_;                 
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; 
    rclcpp::TimerBase::SharedPtr timeout_timer_;     
    ImuDataType current_state_;                     


    const size_t FRAME_LEN = 11;   
    const uint8_t FRAME_HEADER = 0x55;

  
    const float ACC_SCALE = 1.0f / 32768.0f * 16.0f;    
    const float GYRO_SCALE = 1.0f / 32768.0f * 2000.0f; 
    const float ANGLE_SCALE = 1.0f / 32768.0f * 180.0f; 
    const float QUAT_SCALE = 1.0f / 32768.0f;        

    sensor_msgs::msg::Imu imu_msg_; 
};

}

#endif