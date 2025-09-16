#ifndef SERIAL_DEVICE_HPP
#define SERIAL_DEVICE_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace serial_device {
class SerialDevice : public rclcpp::Node {
public:
    // 关键修改：新增 enable_receive_thread 参数（默认true，保持原有逻辑兼容）
    explicit SerialDevice(const std::string& node_name, 
                          const rclcpp::NodeOptions& options, 
                          bool enable_receive_thread = false);

    ~SerialDevice() override;

private:
    void getSerialParams();
    void startAsyncReceive();
    void receiveProcessLoop();

    // 新增：控制是否启用接收线程的开关（构造函数传入）
    bool enable_receive_thread_;
    // 原有接收相关成员
    std::queue<std::vector<uint8_t>> received_data_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> is_receive_thread_running_;
    std::thread receive_process_thread_;

protected:
    // 原有串口资源成员
    std::unique_ptr<drivers::common::IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    void reopenPort();

    virtual void handleReceivedData(const std::vector<uint8_t>& data);
};
}

#endif