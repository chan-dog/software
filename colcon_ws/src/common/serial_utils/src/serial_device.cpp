#include "serial_utils/serial_device.hpp"
#include <rclcpp/node_options.hpp>
#include <chrono>
#include <exception>
#include <sstream>
#include <iomanip>
#include <algorithm> 

namespace serial_device {

SerialDevice::SerialDevice(const std::string& node_name, const rclcpp::NodeOptions& options,  bool enable_receive_thread)
: Node(node_name, options), enable_receive_thread_(enable_receive_thread), is_receive_thread_running_(false), owned_ctx_(std::make_unique<drivers::common::IoContext>(2))
, serial_driver_(std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_))  // 10. serial_driver_
{
RCLCPP_INFO(get_logger(), "Starting SerialDevice Node: %s", node_name.c_str());
RCLCPP_INFO(get_logger(), "Receive thread enabled: %s", enable_receive_thread_ ? "true" : "false");

getSerialParams();  

try {
RCLCPP_INFO(get_logger(), "Initializing serial port with device name: %s", device_name_.c_str());
serial_driver_->init_port(device_name_, *device_config_);

if (!serial_driver_->port()->is_open()) {
RCLCPP_INFO(get_logger(), "Opening serial port: %s", device_name_.c_str());
serial_driver_->port()->open();
RCLCPP_INFO(get_logger(), "Serial port %s opened successfully", device_name_.c_str());
} else {
RCLCPP_INFO(get_logger(), "Serial port %s is already open", device_name_.c_str());
}

if (enable_receive_thread_) {
is_receive_thread_running_ = true;
receive_process_thread_ = std::thread(&SerialDevice::receiveProcessLoop, this);
startAsyncReceive();
RCLCPP_INFO(get_logger(), "Async receive and data process thread started");
}

} catch (const std::exception& ex) {
RCLCPP_ERROR(
get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
throw ex;
}

RCLCPP_INFO(get_logger(), "SerialDevice Node initialization completed");
}

SerialDevice::~SerialDevice()
{
    if (enable_receive_thread_ && is_receive_thread_running_) {
        is_receive_thread_running_ = false;
        queue_cv_.notify_one();  
        if (receive_process_thread_.joinable()) {
            receive_process_thread_.join();
            RCLCPP_INFO(get_logger(), "Data process thread stopped");
        }
    }

    if (serial_driver_ && serial_driver_->port()->is_open()) {
        RCLCPP_INFO(get_logger(), "Closing serial port: %s", device_name_.c_str());
        serial_driver_->port()->close();
        RCLCPP_INFO(get_logger(), "Serial port %s closed", device_name_.c_str());
    }


    if (owned_ctx_) {
        RCLCPP_INFO(get_logger(), "Waiting for I/O context to exit");
        owned_ctx_->waitForExit();
        RCLCPP_INFO(get_logger(), "I/O context exited");
    }

    RCLCPP_INFO(get_logger(), "SerialDevice Node destroyed");
}

void SerialDevice::startAsyncReceive()
{
    if (!enable_receive_thread_) {
        RCLCPP_DEBUG(get_logger(), "Receive thread disabled, skip async receive start");
        return;
    }

    if (!serial_driver_ || !serial_driver_->port() || !serial_driver_->port()->is_open()) {
        RCLCPP_WARN(get_logger(), "Serial port not available, cannot start async receive");
        return;
    }

    auto port = serial_driver_->port();
    // 关键修改：使用const引用参数，驱动自动管理缓冲区
    port->async_receive(
        [this](const std::vector<uint8_t>& received_data, const size_t& actual_size) {
            RCLCPP_DEBUG(get_logger(), "Async receive callback triggered: actual_size=%zu, buffer_size=%zu",
                         actual_size, received_data.size());

            try {
                if (!enable_receive_thread_ || !is_receive_thread_running_) {
                    RCLCPP_DEBUG(get_logger(), "Receive disabled, skip processing");
                    return;
                }

                if (actual_size == 0) {
                    RCLCPP_WARN(get_logger(), "Async receive failed: empty data");
                    reopenPort();
                    if (is_receive_thread_running_ && rclcpp::ok()) {
                        startAsyncReceive();
                    }
                    return;
                }

                std::vector<uint8_t> valid_data(received_data.begin(), received_data.begin() + actual_size);

                // 非空数据入队
                if (!valid_data.empty() && is_receive_thread_running_ && rclcpp::ok()) {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    received_data_queue_.push(valid_data);
                    RCLCPP_DEBUG(get_logger(), "Enqueued %zu bytes, queue length: %zu",
                                 valid_data.size(), received_data_queue_.size());
                    queue_cv_.notify_one();
                }

                if (is_receive_thread_running_ && rclcpp::ok()) {
                    startAsyncReceive();
                }
            } catch (const std::exception& ex) {
                RCLCPP_ERROR(get_logger(), "Receive callback error: %s", ex.what());
                if (is_receive_thread_running_ && rclcpp::ok()) {
                    startAsyncReceive();
                }
            }
        }
    );

    RCLCPP_DEBUG(get_logger(), "Async receive registered (driver-managed buffer)");
}


void SerialDevice::receiveProcessLoop()
{

    if (!enable_receive_thread_) {
        RCLCPP_DEBUG(get_logger(), "Receive thread disabled, skip process loop");
        return;
    }

    RCLCPP_INFO(get_logger(), "Data process loop started");

    while (is_receive_thread_running_ && rclcpp::ok()) {
        std::vector<uint8_t> data_to_process;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() {
                return !received_data_queue_.empty() || !is_receive_thread_running_;
            });

            if (!is_receive_thread_running_ || !rclcpp::ok()) {
                break;
            }

            data_to_process = received_data_queue_.front();
            received_data_queue_.pop();
            RCLCPP_DEBUG(get_logger(), "Data dequeued (size: %zu), queue length: %zu",
                         data_to_process.size(), received_data_queue_.size());
        }

        if (!data_to_process.empty()) {
            handleReceivedData(data_to_process);
        }
    }

    RCLCPP_INFO(get_logger(), "Data process loop exited");
}


void SerialDevice::handleReceivedData(const std::vector<uint8_t>& data)
{
    const size_t MAX_PRINT_LEN = 100;
    size_t print_len = std::min(data.size(), MAX_PRINT_LEN); 

    RCLCPP_INFO(get_logger(), "Received %zu bytes from serial port (print first %zu bytes)",
                data.size(), print_len);

    std::stringstream hex_ss;
    hex_ss << "Hex format (first " << print_len << " bytes): ";
    for (size_t i = 0; i < print_len; ++i) {
        hex_ss << std::hex << std::setw(2) << std::setfill('0') 
               << static_cast<int>(data[i]) << " ";
    }
    // 若数据超过100字节，添加总长度提示
    if (data.size() > MAX_PRINT_LEN) {
        hex_ss << "... (total " << data.size() << " bytes)";
    }
    RCLCPP_INFO(get_logger(), "%s", hex_ss.str().c_str());
}


void SerialDevice::reopenPort()
{
    RCLCPP_WARN(get_logger(), "Attempting to reopen port: %s", device_name_.c_str());
    try {
        if (serial_driver_ && serial_driver_->port()) {
            if (serial_driver_->port()->is_open()) {
                serial_driver_->port()->close();
                RCLCPP_INFO(get_logger(), "Closed port before reopening: %s", device_name_.c_str());
            }

            serial_driver_->port()->open();
            if (serial_driver_->port()->is_open()) {
                RCLCPP_INFO(get_logger(), "Successfully reopened port: %s", device_name_.c_str());
                if (enable_receive_thread_ && is_receive_thread_running_ && rclcpp::ok()) {
                    startAsyncReceive();
                    RCLCPP_INFO(get_logger(), "Async receive restarted after port reopen");
                }
            } else {
                RCLCPP_ERROR(get_logger(), "Port reopen failed: %s is still closed", device_name_.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Serial driver or port is invalid, cannot reopen");
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            reopenPort();
        }
    }
}

void SerialDevice::getSerialParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate {};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    RCLCPP_INFO(get_logger(), "Loading serial port parameters");

    try {
        device_name_ = declare_parameter<std::string>("device_name", "");
        RCLCPP_INFO(get_logger(), "Loaded device name: %s", device_name_.c_str());
    } catch (rclcpp::ParameterTypeException& ex) {
        RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
        throw ex;
    }

    try {
        baud_rate = declare_parameter<int>("baud_rate", 0);
        RCLCPP_INFO(get_logger(), "Loaded baud rate: %u", baud_rate);
    } catch (rclcpp::ParameterTypeException& ex) {
        RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
        throw ex;
    }

    try {
        const auto fc_string = declare_parameter<std::string>("flow_control", "");
        if (fc_string == "none") fc = FlowControl::NONE;
        else if (fc_string == "hardware") fc = FlowControl::HARDWARE;
        else if (fc_string == "software") fc = FlowControl::SOFTWARE;
        else throw std::invalid_argument {
            "The flow_control parameter must be one of: none, software, or hardware."
        };
        RCLCPP_INFO(get_logger(), "Loaded flow control: %s", fc_string.c_str());
    } catch (rclcpp::ParameterTypeException& ex) {
        RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
        throw ex;
    }

    try {
        const auto pt_string = declare_parameter<std::string>("parity", "");
        if (pt_string == "none") pt = Parity::NONE;
        else if (pt_string == "odd") pt = Parity::ODD;
        else throw std::invalid_argument {
            "The parity parameter must be one of: none, odd."
        };
        RCLCPP_INFO(get_logger(), "Loaded parity: %s", pt_string.c_str());
    } catch (rclcpp::ParameterTypeException& ex) {
        RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
        throw ex;
    }

    try {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "");
        if (sb_string == "1" || sb_string == "1.0") sb = StopBits::ONE;
        else if (sb_string == "1.5") sb = StopBits::ONE_POINT_FIVE;
        else if (sb_string == "2" || sb_string == "2.0") sb = StopBits::TWO;
        else throw std::invalid_argument { 
            "The stop_bits parameter must be one of: 1, 1.5, or 2." 
        };
        RCLCPP_INFO(get_logger(), "Loaded stop bits: %s", sb_string.c_str());
    } catch (rclcpp::ParameterTypeException& ex) {
        RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
        throw ex;
    }

    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    RCLCPP_INFO(get_logger(), "Serial port parameters loaded successfully");
}

}