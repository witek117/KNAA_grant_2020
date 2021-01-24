#include "main.h"
#include "TMC2208Stepper.h"
#include "STM_UART.hpp"
#include "STM_GPIO.hpp"

STM_Uart debugUart = {USART1, 115200};

STM32_GPIO LED = {LED_GPIO_Port, LED_Pin};


//TMC2208Stepper TMC = {&tmcUart, 0.11, TMC2208Stepper::TMC2208_SLAVE_ADDR, };

extern "C"
int myMain() {
//    uint8_t k [] = {'0', '2','2', '4', '\n', 0};
    debugUart.init();

    while (true) {

        HAL_Delay(100);
        int len = debugUart.available();
        if (len > 0) {
//            debugUart.write(k, 5);
            debugUart.write(debugUart.read() + 48);
//            debugUart.read();
            LED.toggle();
        }

    }
}
