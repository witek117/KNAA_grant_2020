#pragma once
#include "I2C.hpp"
#include "stm32f1xx.h"

class STM_I2C : public I2C {
    I2C_HandleTypeDef& hi2c;
    uint8_t data_buffer [10] = {0};
    uint8_t address = 0;
    uint8_t indexBuffer = 0;
    uint8_t readIndexBuffer = 0;
public:
    STM_I2C(I2C_HandleTypeDef& hi2c) : hi2c(hi2c) {}

    void init() override {

    }
    void beginTransmission(uint8_t addr) override {
        address = addr;
        indexBuffer = 0;
        readIndexBuffer = 0;
    }

    void write(uint8_t reg) override {
        data_buffer[indexBuffer++] = reg;
    }

    void endTransmission() override {
        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)&hi2c, address, data_buffer, indexBuffer, 10);
    }

    void requestFrom(uint8_t addr, uint8_t length) override {

//        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)&hi2c, , &length, 1, 10);
        HAL_I2C_Master_Receive((I2C_HandleTypeDef*)&hi2c, addr, data_buffer, length, 10);
        indexBuffer = length;
        readIndexBuffer = 0;
    }

    uint8_t read() override {
        if (readIndexBuffer < indexBuffer) {
            return data_buffer[readIndexBuffer++];
        }
        return 0;
    }



//    void write(uint8_t address, uint8_t register_address) override {
//        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)&hi2c, address, (uint8_t*)&register_address, 1, 10);
//    }
//
//    void write(uint8_t address, uint8_t register_address, uint8_t data) override {
//        uint8_t buffer[2] = {uint8_t (register_address), data};
//        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)&hi2c, address, buffer, 2, 10);
//    }
//
//    void write(uint8_t address, uint8_t register_address, uint16_t data) override {
//        uint8_t buffer[3] = {uint8_t (register_address), uint8_t(data >> 8), uint8_t(data)};
//        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)&hi2c, address, buffer, 3, 10);
//    }
//
//    uint8_t read(uint8_t address, uint8_t register_address) override {
//        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)&hi2c, address, (uint8_t*)&register_address, 1, 10);
//        HAL_I2C_Master_Receive((I2C_HandleTypeDef*)&hi2c, address, data_buffer, 1, 10);
//        return data_buffer[0];
//    }
//
//    uint8_t* read(uint8_t address, uint8_t register_address, size_t size) override {
//        HAL_I2C_Master_Transmit((I2C_HandleTypeDef*)&hi2c, address, (uint8_t*)&register_address, 1, 10);
//        HAL_I2C_Master_Receive((I2C_HandleTypeDef*)&hi2c, address, data_buffer, size, 10);
//        return data_buffer;
//    }
};