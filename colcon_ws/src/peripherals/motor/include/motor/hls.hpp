#pragma once

#include "feetech.hpp"
#include "serial_driver/serial_driver.hpp"

namespace feetech {

class HLS : public Servo {
public:
    enum class Mode {
        POSITION = 0x00,
        SPEED = 0x01,
        CURRENT = 0x02,
        PWM = 0x03,
    };

    drivers::serial_driver::SerialDriver* serial;
    HLS(uint8_t id, uint16_t origin_pos ,int16_t max_pos, int16_t min_pos, int16_t speed, int16_t acc, drivers::serial_driver::SerialDriver* serial);

    ~HLS();

    void write(uint8_t id, std::vector<uint8_t> data);
    bool read();

    bool ping() override;
    void setPos(int16_t pos) override;
    void setOffset(uint16_t offset) override;
    int16_t getPos() override;
    uint8_t getCheckSum(Packet packet) const override;
    void setId(uint8_t id) override;
    void action();

    std::vector<uint8_t> serialize(Packet packet) const;
    Packet deserialize(std::vector<uint8_t> data) const;
    uint8_t getID() { return id; };

    void setMode(bool isStore, Mode mode);
    uint8_t getMode();
    void setSpeed(int16_t speed);
    int16_t getSpeed();
    void enable();
    void disable();

    int16_t getMaxSpeed() const { return max_speed; }
    int16_t getMinSpeed() const { return min_speed; }
    // int16_t getOriginPos() const { return origin_pos; }

private:
    int16_t scs_tohost(int16_t pos, int n)
    {
        if (pos & (1 << n))
            return (pos & ~(1 << n)) * -1;
        else
            return pos;
    }
    mutable Packet receive_packet;
    int16_t torque = 200;
    int16_t max_speed = 50;
    int16_t min_speed = -50;
    // int16_t origin_pos = 0;

};
}