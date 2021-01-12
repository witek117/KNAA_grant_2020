#pragma once

class Stream{

public:
    virtual int available() = 0;
    virtual uint8_t read() = 0;
    virtual int write(uint8_t) = 0;
};