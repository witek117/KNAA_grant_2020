#pragma once
#include <cstdint>

class SSwitch {
  public:
    SSwitch(const uint16_t pin1, const uint16_t pin2, const uint8_t address);
    void active();
  private:
    const uint16_t p1;
    const uint16_t p2;
    const uint8_t addr;
};
