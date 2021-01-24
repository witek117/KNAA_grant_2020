#include "main.h"
#include "TMC2208Stepper.h"
#include "STM_UART.hpp"
#include "STM_GPIO.hpp"
#include "SERIAL_SWITCH.h"

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

TMC2208Stepper TMC = {tmcUart, tmcSwitch, 0.11, TMC2208Stepper::TMC2208_SLAVE_ADDR};

extern "C"
int myMain() {
//    uint8_t k [] = {'0', '2','2', '4', '\n', 0};
    debugUart.init();

    while (true) {

        HAL_Delay(100);
//        debugUart.write(k, 5);
        LED1.toggle();
        LED2.toggle();
        LED3.toggle();
        int len = debugUart.available();
        if (len > 0) {
//            debugUart.write(k, 5);
            debugUart.write(debugUart.read() + 48);
//            debugUart.read();

        }

    }
}
