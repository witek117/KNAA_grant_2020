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

STM32_GPIO LED2 = {LED2_GPIO_Port, LED2_Pin};

void enable_interrupts() {}

void disable_interrupts() {}

void print_idn_callback(const char* data);
void test_LED_callback(const char* data);

Command idn("*IDN?", print_idn_callback);
Command test("test", test_LED_callback);

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

STM32_GPIO LED3 = {LED3_GPIO_Port, LED3_Pin};

STM32_GPIO STEP = {DIR_GPIO_Port, DIR_Pin};
//STM32_GPIO DIR = {DIR_GPIO_Port, DIR_Pin};
STM32_GPIO EN = {ENABLE_GPIO_Port, ENABLE_Pin};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    if (GPIO_Pin == GPIO1_Pin) {
        laser.ISR();
    }
}

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
void print_function(uint8_t data) {
    rs485.write(data);
}

CommandManager<10> command_manager(&enable_interrupts, &disable_interrupts, &print_function);

void redirect_handler(uint8_t data) {
    (void)data;
//    LED3.toggle();
    command_manager.reader.putChar((char)data);

//    command_manager.print('d');
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
//    DIR.set();



//    uint8_t k [] = {'0', '2','2', '4', '\n', 0};
    laser.setAddress(laser.getAddress() << 1u);

    debugUart.init();
    laser.init();

    command_manager.addCommand(&idn);
    command_manager.addCommand(&test);


//    rs485.write((uint8_t *) "123456789", 9);

//    debugUart.write_int((int)(laser.getAddress()));

    laser.setTimeout(500);

    // lower the return signal rate limit (default is 0.25 MCPS)
    laser.setSignalRateLimit(0.1);
//    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    laser.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    laser.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    laser.setMeasurementTimingBudget(200000);
    laser.startContinuous(300);
//    laser.readRangeContinuousMillimeters();
//    uint32_t kkk = 0;
    while (true) {
        command_manager.run();

        if(laser.haveNewData()) {
            auto data = laser.readAfterISR();
            (void)data;
//            debugUart.write_int(kkk++);
        }
//        kkk++;
//        debugUart.write_int(kkk);
//        rs485.write_int(kkk);
//        command_manager.print('d');
//        rs485.write((uint8_t *) "123456789\n", 10);
//        debugUart.write((uint8_t *) "123456789\n", 10);
//
//        LED3.toggle();
//        delay(0);
//        delay(0);
        if (!HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin)) {
            auto data = laser.readAfterISR();
            debugUart.write_int(data);
        }

        for (uint16_t i = 500; i>0; i--) {
            STEP.set();
//            TMC.step();
            delayMicroseconds(200);
            STEP.reset();
//            TMC.step();
            delayMicroseconds(200);
        }
        shaft = !shaft;
        TMC.shaft(shaft);
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