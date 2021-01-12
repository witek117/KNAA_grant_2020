#pragma once
#include "TMCStepper.h"
#include "TMC2208Stepper.h"
#include "Stream.h"
#include "SERIAL_SWITCH.h"
#include "TMC2209_bitfields.h"

class TMC2209Stepper : public TMC2208Stepper {
public:
    TMC2209Stepper(Stream * SerialPort, float RS, uint8_t addr) :
            TMC2208Stepper(SerialPort, RS, addr) {}

#if SW_CAPABLE_PLATFORM
    TMC2209Stepper(uint16_t SW_RX_pin, uint16_t SW_TX_pin, float RS, uint8_t addr) :
				TMC2208Stepper(SW_RX_pin, SW_TX_pin, RS, addr) {}
#else
    TMC2209Stepper(uint16_t, uint16_t, float, uint8_t) = delete; // Your platform does not currently support Software Serial
#endif
    void push();

    // R: IOIN
    uint32_t IOIN();
    bool enn();
    bool ms1();
    bool ms2();
    bool diag();
    bool pdn_uart();
    bool step();
    bool spread_en();
    bool dir();
    uint8_t version();

    // W: TCOOLTHRS
    uint32_t TCOOLTHRS();
    void TCOOLTHRS(uint32_t input);

    // W: SGTHRS
    void SGTHRS(uint8_t B);
    uint8_t SGTHRS();

    // R: SG_RESULT
    uint16_t SG_RESULT();

    // W: COOLCONF
    void COOLCONF(uint16_t B);
    uint16_t COOLCONF();
    void semin(uint8_t B);
    void seup(uint8_t B);
    void semax(uint8_t B);
    void sedn(uint8_t B);
    void seimin(bool B);
    uint8_t semin();
    uint8_t seup();
    uint8_t semax();
    uint8_t sedn();
    bool seimin();

protected:
    TCOOLTHRS_t TCOOLTHRS_register = TCOOLTHRS_t{.sr=0};
    TMC2209_n::SGTHRS_t SGTHRS_register{.sr=0};
    TMC2209_n::COOLCONF_t COOLCONF_register{{.sr=0}};
};

