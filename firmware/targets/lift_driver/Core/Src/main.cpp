#include "main.h"
#include "TMC2209Stepper.h"
#include "STM_UART.hpp"
#include "STM_GPIO.hpp"
#include "SERIAL_SWITCH.h"
#include "VL53L0X.h"
#include "STM_I2C.h"

void delay(uint32_t time) {
    HAL_Delay(time);
}

int millis() {
    return HAL_GetTick();
}



STM_Uart debugUart = {USART1, 115200};
STM_Uart tmcUart = {USART3, 115200};

class STM_Switch : public TMC_SerialSwitch {
public:
    void active() override {}
    void disactive() override {}
};

STM_Switch tmcSwitch;

extern I2C_HandleTypeDef hi2c1;
STM_I2C I2C = {hi2c1};

VL53L0X laser = {I2C, };


STM32_GPIO LED1 = {LED1_GPIO_Port, LED1_Pin};
STM32_GPIO LED2 = {LED2_GPIO_Port, LED2_Pin};
STM32_GPIO LED3 = {LED3_GPIO_Port, LED3_Pin};

STM32_GPIO STEP = {STEP_GPIO_Port, STEP_Pin};
STM32_GPIO DIR = {DIR_GPIO_Port, DIR_Pin};
STM32_GPIO EN = {ENABLE_GPIO_Port, ENABLE_Pin};



TMC2209Stepper TMC = {tmcUart, tmcSwitch, 0.11, 0x00};

#define STALL_VALUE     100 // [0..255]


void delayMicroseconds(uint32_t time) {
    for (volatile uint32_t i = 0; i < time; ++i) {
        for (volatile uint32_t j = 0; j < 30; j++) {
            ;
        }
    }
}
bool shaft = false;

extern "C"
int myMain() {
    TMC.begin();
    TMC.toff(4);
    TMC.blank_time(24);
    TMC.rms_current(400); // mA
    TMC.microsteps(4);
    TMC.TCOOLTHRS(0xFFFFF); // 20bit max
    TMC.semin(5);
    TMC.semax(2);
    TMC.sedn(0b01);
    TMC.SGTHRS(STALL_VALUE);
    TMC.pwm_autoscale(true);


    EN.reset();
    DIR.set();

//    uint8_t k [] = {'0', '2','2', '4', '\n', 0};
    debugUart.init();

    while (true) {
        for (uint16_t i = 5000; i>0; i--) {
            STEP.set();
            delayMicroseconds(200);
            STEP.reset();
            delayMicroseconds(200);
        }
//        TMC.index_step(1000);
//        delay(1000);
        shaft = !shaft;
        TMC.shaft(shaft);


//        HAL_Delay(100);
////        debugUart.write(k, 5);
        LED1.toggle();
//        LED2.toggle();
//        LED3.toggle();
//        int len = debugUart.available();
//        if (len > 0) {
////            debugUart.write(k, 5);
//            debugUart.write(debugUart.read() + 48);
////            debugUart.read();
//
//        }

    }
}
