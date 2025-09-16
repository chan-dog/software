#include <iostream>
#include <thread>

#include "motor/hls.hpp"

#define DEBUG 0

namespace feetech {
HLS::HLS(uint8_t id, uint16_t origin_pos, int16_t max_pos, int16_t min_pos, int16_t speed, int16_t acc, drivers::serial_driver::SerialDriver* serial)
    : Servo(id)
    , serial(serial)
{
    (void) origin_pos;
    this->id = id;
    this->max_pos = max_pos;
    this->min_pos = min_pos;
    this->speed = speed;
    this->acc = acc;

    if (!serial) {
        throw std::invalid_argument("HLS constructor: serial driver is null");
    }
}

HLS::~HLS() { }

std::vector<uint8_t> HLS::serialize(Packet packet) const
{
    std::vector<uint8_t> data_;
    data_.push_back(packet.header1);
    data_.push_back(packet.header2);
    data_.push_back(packet.id);
    data_.push_back(packet.length);
    data_.push_back(packet.command);
    data_.insert(data_.end(), packet.data.begin(), packet.data.end());
    data_.push_back(packet.checksum);
    return data_;
}

Packet HLS::deserialize(std::vector<uint8_t> data_) const
{
    Packet packet_;
    packet_.header1 = data_[0];
    packet_.header2 = data_[1];
    packet_.id = data_[2];
    packet_.length = data_[3];
    packet_.command = data_[4];
    if (packet_.command != 0) {
        std::cout << "Error is " << (int)packet_.command << std::endl;
        return packet_;
    }
    packet_.data = std::vector<uint8_t>(data_.begin() + 5, data_.end() - 1);
    packet_.checksum = data_.back();
    if (packet_.checksum != data_.back()) {
        std::cout << "Checksum is not correct" << std::endl;
        return packet_;
    }

    return packet_;
}

uint8_t HLS::getCheckSum(Packet packet) const
{
    uint8_t checksum_ = 0;
    checksum_ += packet.id;
    checksum_ += packet.length;
    checksum_ += packet.command;
    if (packet.data.size() > 0) {
        for (int i = 0; i < int(packet.data.size()); i++) {
            checksum_ += packet.data[i];
        }
    }
    checksum_ = (~checksum_) & 0xFF;
    packet.checksum = checksum_;
    if (DEBUG) {
        std::cout << "Checksum is " << (int)packet.checksum << std::endl;
    }
    return checksum_;
}

void HLS::write(uint8_t id, std::vector<uint8_t> data_)
{
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::REGWRITE_DATA);
    packet_.data = data_;
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    std::vector<uint8_t> data = serialize(packet_);

    if (serial->port()->is_open()) {
        serial->port()->send(data);
    }
}

bool HLS::read()
{
    std::vector<uint8_t> data_(18);
    size_t bytes_read = serial->port()->receive(data_);

    if (bytes_read > 0) {
        if (DEBUG) {
            std::cout << "Received data: ";
            for (size_t i = 0; i < bytes_read; ++i) {
                printf("%02X ", data_[i]);
            }
            std::cout << std::endl;
        }
        receive_packet = deserialize(data_);
        if (receive_packet.header1 != PROTOCOL_HEADER1 || receive_packet.header2 != PROTOCOL_HEADER2) {
            std::cout << "Error is " << (int)receive_packet.command << std::endl;
            return false;
        }
    } else {
        std::cout << "No data received or timeout" << std::endl;
        return false;
    }
    return true;
}

void HLS::enable()
{
    std::cout << "Enabling" << std::endl;
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::WRITE);
    packet_.data.push_back(0x28);
    packet_.data.push_back(0x01);
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    if (serial->port()->is_open()) {
        serial->port()->send(serialize(packet_));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void HLS::disable()
{
    std::cout << "Disabling" << std::endl;
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::WRITE);
    packet_.data.push_back(0x28);
    packet_.data.push_back(0x00);
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    if (serial->port()->is_open()) {
        serial->port()->send(serialize(packet_));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void HLS::setId(uint8_t id_)
{
    // 掉电保存
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::WRITE);
    packet_.data.push_back(0x37);
    packet_.data.push_back(0x00);
    packet_.checksum = getCheckSum(packet_);
    packet_.length = packet_.data.size() + 2;

    std::vector<uint8_t> data = serialize(packet_);
    if (serial->port()->is_open()) {
        serial->port()->send(data);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    packet_.data.clear();

    packet_.data.push_back(0x05);
    packet_.data.push_back(id_);
    packet_.checksum = getCheckSum(packet_);
    data = serialize(packet_);
    if (serial->port()->is_open()) {
        serial->port()->send(data);
    }
}

void HLS::setMode(bool isStore, Mode mode)
{

    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::WRITE);

    std::vector<uint8_t> data;
    if (isStore) {
        // 掉电保存
        packet_.data.push_back(0x37);
        packet_.data.push_back(0x00);
        packet_.length = packet_.data.size() + 2;
        packet_.checksum = getCheckSum(packet_);

        data = serialize(packet_);

        if (DEBUG) {
            std::cout << "Sending data: ";
            for (auto byte : data) {
                printf("%02X ", byte);
            }
            std::cout << std::endl;
        }

        if (serial->port()->is_open()) {
            serial->port()->send(data);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!read()) {
            std::cout << "Sending data failed!" << std::endl;
        }
        packet_.data.clear();
    }

    packet_.data.push_back(0x21);
    packet_.data.push_back(static_cast<uint8_t>(mode));
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    data = serialize(packet_);

    if (serial->port()->is_open()) {
        serial->port()->send(data);
    }

    if (!read()) {
        std::cout << "Set mode failed!" << std::endl;
    }

    std::cout << "Set mode successfully!" << std::endl;
}

uint8_t HLS::getMode()
{
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::READ);
    packet_.data.push_back(0x21);
    packet_.data.push_back(0x01);
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    std::vector<uint8_t> data = serialize(packet_);

    if (DEBUG) {
        std::cout << "Sending data: ";
        for (auto byte : data) {
            printf("%02X ", byte);
        }
        std::cout << std::endl;
    }

    if (serial->port()->is_open()) {
        serial->port()->send(data);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (!read()) {
        std::cout << "Get mode failed" << std::endl;
        return static_cast<uint8_t>(feetech::HLS::Mode::CURRENT);
    }

    std::cout << "data size: " << receive_packet.data.size() << std::endl;
    if (!receive_packet.data.empty()) {
        std::cout << receive_packet.data[0] << "data" << std::endl;
        std::cout << "Mode is " << static_cast<int>(receive_packet.data[0]) << std::endl;
        return static_cast<int>(receive_packet.data[0]);
    } else {
        std::cout << "No data received!" << std::endl;
        return 0;
    }
}

bool HLS::ping()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.length = 0x02;
    packet_.command = static_cast<uint8_t>(feetech::Command::PING);
    packet_.checksum = getCheckSum(packet_);

    // std::cout << "Sending ping to servo ID: " << (int)id << std::endl;

    if (serial->port()->is_open()) {
        std::vector<uint8_t> data = serialize(packet_);
        if (DEBUG) {
            std::cout << "Sending data: ";
            for (auto byte : data) {
                printf("%02X ", byte);
            }
            std::cout << std::endl;
        }
        serial->port()->send(data);
    } else {
        std::cout << "Serial port is not open!" << std::endl;
        return false;
    }

    if (!read()) {
        std::cout << "Ping failed - no response from servo" << std::endl;
        return false;
    }

    // std::cout << "Ping successful!" << std::endl;
    return true;
}

void HLS::setSpeed(int16_t speed)
{
    this->speed = abs(speed);

    Packet packet_;
    std::vector<uint8_t> data_;
    data_.push_back(0x29);
    data_.push_back(this->acc);
    data_.push_back(00);
    data_.push_back(00);
    data_.push_back(this->torque & 0xFF);
    data_.push_back(this->torque >> 8 & 0xFF);
    data_.push_back(this->speed & 0xFF);
    data_.push_back(this->speed >> 8 & 0xFF);
  if (speed < 0) {
    data_.back() |= 0x80;     // 设置最高位为1表示负速度
  }

    if (serial->port()->is_open()) {
        write(id, data_);
    }

    if (!read()) {
        std::cout << "Set speed failed" << std::endl;
        return;
    }

    if (DEBUG) {
        std::cout << "Set speed successful" << std::endl;
    }
}

void HLS::setPos(int16_t pos)
{
    if (pos > max_pos) {
        pos = max_pos;
    }
    if (pos < min_pos) {
        pos = min_pos;
    }

    // pos = scs_tohost(pos,15);

    this->pos = pos;

    Packet packet_;
    std::vector<uint8_t> data_;
    data_.push_back(0x29);
    data_.push_back(this->acc);
    data_.push_back(this->pos & 0xFF);
    data_.push_back(this->pos >> 8 & 0xFF);
    data_.push_back(this->torque & 0xFF);
    data_.push_back(this->torque >> 8 & 0xFF);
    data_.push_back(this->speed & 0xFF);
    data_.push_back(this->speed >> 8 & 0xFF);

    if (serial->port()->is_open()) {
        write(id, data_);
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if (!read()) {
        std::cout << "Write failed" << std::endl;
        return;
    }

    if (DEBUG) {
        std::cout << "Write successful" << std::endl;
    }
}

void HLS::setOffset(uint16_t offset)
{
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::RESET);
    packet_.data.push_back(offset & 0xFF);
    packet_.data.push_back(offset >> 8);
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    std::vector<uint8_t> data_ = serialize(packet_);

    if (DEBUG) {
        std::cout << "Sending data: ";
        for (auto byte : data_) {
            printf("%02X ", byte);
        }
        std::cout << std::endl;
    }

    if (serial->port()->is_open()) {
        serial->port()->send(data_);
    }

    if (!read()) {
        std::cout << "Set offset failed" << std::endl;
        return;
    }
}

int16_t HLS::getSpeed()
{
    std::cout << "Getting Speed" << std::endl;
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::READ);
    packet_.data.push_back(0x3A);
    packet_.data.push_back(0x02);
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    std::vector<uint8_t> data_ = serialize(packet_);

    if (DEBUG) {
        std::cout << "Sending data: ";
        for (auto byte : data_) {
            printf("%02X ", byte);
        }
        std::cout << std::endl;
    }

    if (serial->port()->is_open()) {
        serial->port()->send(data_);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if (!read()) {
        std::cout << "Failed to read position" << std::endl;
        return 0;
    }
    int16_t speed_ = receive_packet.data[0] | (receive_packet.data[1] << 8);
    receive_packet.data.clear();
    // pos = scs_tohost(pos,15);
    std::cout << "Speed is " << speed << std::endl;
    return speed_;
}

int16_t HLS::getPos()
{
    // std::cout << "Getting position" << std::endl;
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = id;
    packet_.command = static_cast<uint8_t>(feetech::Command::READ);
    packet_.data.push_back(0x38);
    packet_.data.push_back(0x02);
    packet_.length = packet_.data.size() + 2;
    packet_.checksum = getCheckSum(packet_);

    std::vector<uint8_t> data_ = serialize(packet_);

    if (DEBUG) {
        std::cout << "Sending data: ";
        for (auto byte : data_) {
            printf("%02X ", byte);
        }
        std::cout << std::endl;
    }

    if (serial->port()->is_open()) {
        serial->port()->send(data_);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if (!read()) {
        std::cout << "Failed to read position" << std::endl;
        return 0;
    }
    int16_t pos = receive_packet.data[0] | (receive_packet.data[1] << 8);
    receive_packet.data.clear();
    // pos = scs_tohost(pos,15);
    return pos;
}

void HLS::action()
{
    Packet packet_;
    packet_.header1 = PROTOCOL_HEADER1;
    packet_.header2 = PROTOCOL_HEADER2;
    packet_.id = 0xFE;
    packet_.length = 0x02;
    packet_.command = static_cast<uint8_t>(feetech::Command::ACTION);
    packet_.checksum = getCheckSum(packet_);

    if (DEBUG) {
        std::cout << "Sending data: ";
        for (auto byte : serialize(packet_)) {
            printf("%02X ", byte);
        }
        std::cout << std::endl;
    }
    if (serial->port()->is_open()) {
        serial->port()->send(serialize(packet_));
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
}
