#pragma once
#include <cstdint>

class TMC_SerialSwitch {
public:
    virtual void active() = 0;
    virtual void disactive() = 0;
};
