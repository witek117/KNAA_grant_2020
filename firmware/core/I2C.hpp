#pragma once


class I2C{

public:
    virtual void init() = 0;
    virtual void beginTransmission(uint8_t address) = 0;
    virtual void write(uint8_t reg) = 0;
    virtual void endTransmission() = 0;
    virtual void requestFrom(uint8_t address, uint8_t count) = 0;
    virtual uint8_t read() = 0;
};