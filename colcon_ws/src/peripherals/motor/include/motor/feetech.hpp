#pragma once

#include <cstdint>
#include <vector>
namespace feetech {

// 定义协议头
#define PROTOCOL_HEADER1 0xFF
#define PROTOCOL_HEADER2 0xFF

// 定义协议指令集
enum class Command {
    PING = 0x01, // 查询
    READ = 0x02, // 读取
    WRITE = 0x03, // 写入
    REGWRITE_DATA = 0x04, // 异步写入
    ACTION = 0x05, // 执行
    REBOOT = 0x08, // 重启
    RESET = 0x0B, // postion reset
    SYCNREAD_DATA = 0x82, // 同步读取
    SYCNWRITE_DATA = 0x83 // 同步写入
};

struct Packet {
    uint8_t header1;
    uint8_t header2;
    uint8_t id;
    uint8_t length;
    uint8_t command;
    std::vector<uint8_t> data;
    uint8_t checksum;
};

class Servo {
public:
    Servo(uint8_t id)
        : id(id) { };
    virtual ~Servo() { };

    virtual bool ping() { return false; }
    virtual void setPos(int16_t pos) { this->pos = pos; }
    virtual void setOffset(uint16_t offset) { (void)offset; }
    virtual int16_t getPos() { return pos; }
    virtual uint8_t getCheckSum(Packet packet) const { (void)packet; return 0; }
    virtual void setId(uint8_t id) { this->id = id; }

protected:
    uint8_t id;
    int16_t pos;
    int16_t max_pos;
    int16_t min_pos;
    int16_t speed;
    uint8_t acc;
};

} // namespace feetech