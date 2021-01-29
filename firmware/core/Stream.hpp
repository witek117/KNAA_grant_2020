#pragma once

#include <cstdint>

class Stream{

public:
    virtual int available() = 0;
    virtual uint8_t read() = 0;
    virtual int write(uint8_t) = 0;
    virtual int write(uint8_t*, uint16_t len) = 0;
    virtual void init() = 0;
};