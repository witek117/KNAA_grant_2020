#include "main.h"
#include "TMC2208Stepper.h"
#include "STM_UART.hpp"
#include "STM_GPIO.hpp"


//extern UART_HandleTypeDef huart1;
STM_Uart debugUart = {USART1, 115200};

STM32_GPIO LED = {LED_GPIO_Port, LED_Pin};


//TMC2208Stepper TMC = {&tmcUart, 0.11, TMC2208Stepper::TMC2208_SLAVE_ADDR, };

extern "C"
int myMain() {
//     LED1.reset();
//     uint8_t k = 5;
//     HAL_UART_Transmit(&huart1, &k, 1, 10);
    uint8_t k [] = {'0', '2','2', '4', '\n', 0};
//
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    /*Configure GPIO pin : LED_Pin */
//    GPIO_InitStruct.Pin = LED_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
//
//    /*Configure GPIO pin : LED_Pin */
//    GPIO_InitStruct.Pin = LED_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

     debugUart.init();

//     debugUart.write('d');

    while (true) {
        debugUart.write(k, 5);
//        HAL_UART_Transmit(&huart1, k, 5, 100);
        HAL_Delay(100);
        LED.toggle();
//        debugUart.write('d');
    }
}
