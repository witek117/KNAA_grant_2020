#pragma once

#include <stm32f1xx.h>
#include <cstdint>
#include "GPIO.hpp"

class STM32_GPIO : public GPIO {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
public:
    STM32_GPIO (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) : GPIOx(GPIOx), GPIO_Pin(GPIO_Pin) {

    }

    void set() override {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    }

    void reset() override {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    }

    void toggle() override {
        HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
    }

    bool get() override {
        return static_cast<bool>(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin));
    }
};
