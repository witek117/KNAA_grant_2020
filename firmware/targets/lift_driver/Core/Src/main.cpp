#include "main.h"
#include "TMC2209Stepper.h"
#include "STM_UART.hpp"
#include "STM_GPIO.hpp"
#include "SERIAL_SWITCH.h"
#include "VL53L0X.h"
#include "STM_I2C.h"
#include <cstring>
#include "Command.h"
#include "command_manager.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void enable_interrupts() { __enable_irq(); }
void disable_interrupts() { __disable_irq(); }
void print_function(uint8_t data);
void print_idn_callback(const char* data);
void test_LED_callback(const char* data);
void move_callback(const char* data);

Command idn("*IDN?", print_idn_callback);
Command test("test", test_LED_callback);
Command move("move", move_callback);
CommandManager<10> command_manager(&enable_interrupts, &disable_interrupts, &print_function);

void delay(uint32_t time) {
    HAL_Delay(time);
}

int millis() {
    return HAL_GetTick();
}

STM32_GPIO DE = {DE_GPIO_Port, DE_Pin};
STM_RS485 rs485 = {USART2, 115200, DE};
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
VL53L0X laser = {I2C };

STM32_GPIO LED1 = {LED1_GPIO_Port, LED1_Pin};
STM32_GPIO LED2 = {LED2_GPIO_Port, LED2_Pin};
STM32_GPIO LED3 = {LED3_GPIO_Port, LED3_Pin};

STM32_GPIO EN = {ENABLE_GPIO_Port, ENABLE_Pin};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    if (GPIO_Pin == GPIO1_Pin) {
        laser.ISR();
    }
}

TMC2209Stepper TMC = {tmcUart, tmcSwitch, 0.11, 0x00};

#define STALL_VALUE     100 // [0..255]

void print_function(uint8_t data) {
    rs485.write(data);
}

void redirect_handler(uint8_t data) {
    command_manager.reader.putChar((char)data);
}

extern "C"
[[noreturn]] int myMain() {
    command_manager.init();
    rs485.init();
    rs485.setRedirectHandler(redirect_handler);

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

    debugUart.init();

    command_manager.addCommand(&idn);
    command_manager.addCommand(&test);
    command_manager.addCommand(&move);

    laser.setAddress(laser.getAddress() << 1u);
    laser.init();
    laser.setTimeout(500);

    // lower the return signal rate limit (default is 0.25 MCPS)
    laser.setSignalRateLimit(0.1);
//    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    laser.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    laser.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    laser.setMeasurementTimingBudget(200000);
    laser.startContinuous(300);

    while (true) {
        command_manager.run();
        if(laser.haveNewData()) {
            auto data = laser.readAfterISR();
            (void)data;
        }

    }
}

void print_idn_callback(const char* data) {
    (void)data;
    command_manager.print("Lift Driver v1.0\n");
}

void test_LED_callback(const char* data) {
    (void)data;
    LED1.set(); HAL_Delay(100);
    LED2.set(); HAL_Delay(100);
    LED3.set(); HAL_Delay(100);

    LED1.reset(); HAL_Delay(100);
    LED2.reset(); HAL_Delay(100);
    LED3.reset();
}

void move_callback(const char* data){
    auto [move_step] = parser::get<int>(data);
    if (move_step > 0) {
        TMC.shaft(true);
    } else {
        TMC.shaft(false);
        move_step = std::abs(move_step);
    }

    htim3.Instance->ARR = move_step;
    htim1.Instance->CNT = 0;
    htim3.Instance->CNT = 0;

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
}
extern "C"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM3) {
        HAL_TIM_Base_Stop(&htim3);
        HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
    }
}

