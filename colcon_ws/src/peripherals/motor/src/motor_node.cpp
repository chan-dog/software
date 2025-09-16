#include "motor/motor_node.hpp"
#include <rclcpp/node_options.hpp>
#include <serial_utils/serial_device.hpp>

namespace serial_device {
Motor::Motor(const rclcpp::NodeOptions& options)
    : SerialDevice("motor_node", options)
{
    getMotorParams();
    initDevices();

    // Create Subscription
    cmd_vel_sub_ = this->create_subscription<robot_interfaces::msg::HeadCmd>(
        "/cmd_vel/head", rclcpp::SensorDataQoS(),
        std::bind(&Motor::sendData, this, std::placeholders::_1));
}

void Motor::getMotorParams()
{
    try {
        const auto motor_list = this->declare_parameter<std::vector<std::string>>("motor_list");

        for (const auto& entry : motor_list) {
            std::istringstream ss(entry);
            std::string name;
            int id, origin_pos, max_pos, min_pos, speed, acc;

            ss >> name >> id >> origin_pos >> max_pos >> min_pos >> speed >> acc;
            if (ss.fail()) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse motor entry: %s", entry.c_str());
                continue;
            }

            motors_[name] = std::make_unique<feetech::HLS>(
                id, origin_pos, max_pos, min_pos, speed, acc, serial_driver_.get());

            RCLCPP_INFO(this->get_logger(), "Loaded motor [%s]: id=%d", name.c_str(), id);
        }
    } catch (rclcpp::ParameterTypeException& ex) {
        RCLCPP_ERROR(get_logger(), "The motor_list provided was invalid");
        throw ex;
    }
}

void Motor::initDevices()
{
    try {
        for (auto& [id, motor] : motors_) {
            if (motor) {
                RCLCPP_INFO(get_logger(), "Motor %s initialized with ID %d", id.c_str(), motor->getID());
                if (motor->ping()) {
                    RCLCPP_INFO(get_logger(), "Motor %s is responsive", id.c_str());
                } else {
                    RCLCPP_ERROR(get_logger(), "Motor %s is not responsive", id.c_str());
                }
            } else {
                RCLCPP_ERROR(get_logger(), "Motor %s initialization failed", id.c_str());
            }
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "Error initializing motors: %s", ex.what());
        throw ex;
    }

    RCLCPP_INFO(get_logger(), "All motors initialized successfully");
}

void Motor::sendData(const robot_interfaces::msg::HeadCmd::SharedPtr msg)
{
    for (const auto& cmd : msg->cmds) {
        const auto& name = cmd.name;
        const auto& target = cmd.target;
        const auto& mode = cmd.mode;
        const auto& enable = cmd.enable;
        const auto& reset = cmd.reset;

        auto it = motors_.find(name);
        if (it != motors_.end() && it->second) {
            if (!enable) {
                it->second->disable();
                // continue;
            } else {
                if (mode == (int)feetech::HLS::Mode::POSITION) {
                    if (!reset) {
#if DEBUG
                        RCLCPP_INFO(get_logger(), "Setting motor [%s] position to %f", name.c_str(), target);
                        RCLCPP_INFO(get_logger(), "Servo %s Target position: %d", name.c_str(), it->second->getPos());
#endif
                        it->second->setPos(it->second->getPos() + target * 2048 / M_PI);
                        // it->second->setPos(target);
                    } else {
#if DEBUG
                        RCLCPP_INFO(get_logger(), "Resetting motor [%s] to origin position", name.c_str());
#endif
                        it->second->setPos(2048);
                        it->second->action();
                    }
                } else if (mode == (int)feetech::HLS::Mode::SPEED) {
                    if (target > it->second->getMaxSpeed() || target < it->second->getMinSpeed()) {
                        RCLCPP_ERROR(get_logger(), "Speed out of range for motor [%s]: %f", name.c_str(), target);
                    } else {
                        if (!reset) {
                            if (!(it->second->getMode() == (int)feetech::HLS::Mode::SPEED)) {
#if DEBUG
                                RCLCPP_INFO(get_logger(), "Resetting motor [%s] to origin position", name.c_str());
#endif
                                it->second->setMode(false, feetech::HLS::Mode::SPEED);
                            }
                            RCLCPP_INFO(get_logger(), "Setting motor [%s] speed to %f", name.c_str(), target);
                            it->second->setSpeed(target);
                        } else {
#if DEBUG
                            RCLCPP_INFO(get_logger(), "Setting motor [%s] mode to SPEED", name.c_str());
                            ;
#endif
                            it->second->setMode(false, feetech::HLS::Mode::POSITION);
                            it->second->setPos(2048);

                            it->second->action();
                        }
                    }
                }
            }
        } else {
            RCLCPP_WARN(get_logger(), "Motor with name '%s' not found.", name.c_str());
        }
    }

    if (!motors_.empty())
        motors_.begin()->second->action();
}

} // namespace serial_device

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(serial_device::Motor)