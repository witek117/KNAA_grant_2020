#include "main.h"
#include "TMC2209Stepper.h"
#include "STM_UART.hpp"
#include "STM_GPIO.hpp"
#include "SERIAL_SWITCH.h"

void delay(uint32_t time) {
    HAL_Delay(time);
}

int millis() {
    return HAL_GetTick();
}

STM_Uart debugUart = {USART1, 115200};
STM_Uart tmcUart = {USART2, 115200};

class STM_Switch : public TMC_SerialSwitch {
public:
    void active() override {}
    void disactive() override {}
};

STM_Switch tmcSwitch;

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
        for (volatile uint32_t j = 0; j < 10; j++) {
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
    TMC.microsteps(0);
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
//        for (uint16_t i = 5000; i>0; i--) {
//            STEP.set();
//            delayMicroseconds(160);
//            STEP.reset();
//            delayMicroseconds(160);
//        }
//        shaft = !shaft;
//        TMC.shaft(shaft);


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
