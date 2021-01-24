#pragma once
#include "TMCStepper.h"
#include "TMC2208Stepper.h"
#include "Stream.hpp"
#include "SERIAL_SWITCH.h"
#include "TMC2209_bitfields.h"

class TMC2209Stepper : public TMC2208Stepper {
public:
    TMC2209Stepper(Stream &SerialPort, TMC_SerialSwitch &SerialSwitch, float RS, uint8_t addr) :
            TMC2208Stepper(SerialPort, SerialSwitch, RS, addr) {}

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

