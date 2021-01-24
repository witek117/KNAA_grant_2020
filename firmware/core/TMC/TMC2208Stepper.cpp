#include "TMC2208Stepper.h"

#include "Stream.hpp"
#include "HAL.hpp"
#include <cstdint>

void TMC2208Stepper::begin() {
	pdn_disable(true);
	mstep_reg_select(true);
	HWSerial.init();
}

void TMC2208Stepper::defaults() {
	GCONF_register.i_scale_analog = 1;
	GCONF_register.internal_rsense = 0; // OTP
	GCONF_register.en_spreadcycle = 0; // OTP
	GCONF_register.multistep_filt = 1; // OTP
	IHOLD_IRUN_register.iholddelay = 1; // OTP
	TPOWERDOWN_register.sr = 20;
	CHOPCONF_register.sr = 0x10000053;
	PWMCONF_register.sr = 0xC10D0024;
  //MSLUT0_register.sr = ???;
  //MSLUT1_register.sr = ???;
  //MSLUT2_register.sr = ???;
  //MSLUT3_register.sr = ???;
  //MSLUT4_register.sr = ???;
  //MSLUT5_register.sr = ???;
  //MSLUT6_register.sr = ???;
  //MSLUT7_register.sr = ???;
  //MSLUTSTART_register.start_sin90 = 247;
}

void TMC2208Stepper::push() {
	GCONF(GCONF_register.sr);
	IHOLD_IRUN(IHOLD_IRUN_register.sr);
//	SLAVECONF(SLAVECONF_register.sr);
	TPOWERDOWN(TPOWERDOWN_register.sr);
	TPWMTHRS(TPWMTHRS_register.sr);
	VACTUAL(VACTUAL_register.sr);
	CHOPCONF(CHOPCONF_register.sr);
	PWMCONF(PWMCONF_register.sr);
}

bool TMC2208Stepper::isEnabled() { return !enn() && toff(); }

uint8_t TMC2208Stepper::calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		}
	}
	return crc;
}

__attribute__((weak))
int TMC2208Stepper::available() {
    return HWSerial.available();
}

__attribute__((weak))
void TMC2208Stepper::preWriteCommunication() {
    SerialSwitch.active();
}

__attribute__((weak))
void TMC2208Stepper::preReadCommunication() {
    SerialSwitch.active();
}

__attribute__((weak))
int16_t TMC2208Stepper::serial_read() {
	return HWSerial.read();
}

__attribute__((weak))
uint8_t TMC2208Stepper::serial_write(const uint8_t data) {
    return HWSerial.write(data);
}

__attribute__((weak))
uint8_t TMC2208Stepper::serial_write(uint8_t *data, uint8_t len) {
    return HWSerial.write(data, len);
}

__attribute__((weak))
void TMC2208Stepper::postWriteCommunication() {
    SerialSwitch.disactive();
}

__attribute__((weak))
void TMC2208Stepper::postReadCommunication() {
    SerialSwitch.disactive();
}

void TMC2208Stepper::write(uint8_t addr, uint32_t regVal) {
	uint8_t len = 7;
	addr |= TMC_WRITE;
	uint8_t datagram[] = {TMC2208_SYNC, slave_address, addr, (uint8_t)(regVal>>24), (uint8_t)(regVal>>16), (uint8_t)(regVal>>8), (uint8_t)(regVal>>0), 0x00};

	datagram[len] = calcCRC(datagram, len);

	preWriteCommunication();

//	for(uint8_t i=0; i<=len; i++) {
//		serial_write(datagram[i]);
//	}

	serial_write(datagram, len + 1);

	postWriteCommunication();

	delay(replyDelay);
}

uint64_t TMC2208Stepper::_sendDatagram(uint8_t datagram[], const uint8_t len, uint16_t timeout) {
	while (available() > 0) serial_read(); // Flush

//	#if defined(ARDUINO_ARCH_AVR)
//		if (RXTX_pin > 0) {
//			digitalWrite(RXTX_pin, HIGH);
//			pinMode(RXTX_pin, OUTPUT);
//		}
//	#endif

//	for(int i=0; i<=len; i++) serial_write(datagram[i]);

	serial_write(datagram, len + 1);

//	#if defined(ARDUINO_ARCH_AVR)
//		if (RXTX_pin > 0) {
//			pinMode(RXTX_pin, INPUT_PULLUP);
//		}
//	#endif

	delay(replyDelay);

	// scan for the rx frame and read it
	uint32_t ms = millis();
	uint32_t sync_target = (static_cast<uint32_t>(datagram[0])<<16) | 0xFF00u | datagram[2];
	uint32_t sync = 0;

	do {
		uint32_t ms2 = millis();
		if (ms2 != ms) {
			// 1ms tick
			ms = ms2;
			timeout--;
		}
		if (!timeout) return 0;

		int16_t res = serial_read();
		if (res < 0) continue;

		sync <<= 8u;
		sync |= res & 0xFFu;
		sync &= 0xFFFFFFu;

	} while (sync != sync_target);

	uint64_t out = sync;
	ms = millis();
	timeout = abort_window;

	for(uint8_t i=0; i<5;) {
		uint32_t ms2 = millis();
		if (ms2 != ms) {
			// 1ms tick
			ms = ms2;
			timeout--;
		}
		if (!timeout) return 0;

		int16_t res = serial_read();
		if (res < 0) continue;

		out <<= 8u;
		out |= res & 0xFFu;

		i++;
	}

//	#if defined(ARDUINO_ARCH_AVR)
//		if (RXTX_pin > 0) {
//			digitalWrite(RXTX_pin, HIGH);
//			pinMode(RXTX_pin, OUTPUT);
//		}
//	#endif

	while (available() > 0) serial_read(); // Flush

	return out;
}

uint32_t TMC2208Stepper::read(uint8_t addr) {
	constexpr uint8_t len = 3;
	addr |= TMC_READ;
	uint8_t datagram[] = {TMC2208_SYNC, slave_address, addr, 0x00};
	datagram[len] = calcCRC(datagram, len);
	uint64_t out = 0x00000000UL;

	for (uint8_t i = 0; i < max_retries; i++) {
		preReadCommunication();
		out = _sendDatagram(datagram, len, abort_window);
		postReadCommunication();

		delay(replyDelay);

		CRCerror = false;
		uint8_t out_datagram[] = {
			static_cast<uint8_t>(out>>56u),
			static_cast<uint8_t>(out>>48u),
			static_cast<uint8_t>(out>>40u),
			static_cast<uint8_t>(out>>32u),
			static_cast<uint8_t>(out>>24u),
			static_cast<uint8_t>(out>>16u),
			static_cast<uint8_t>(out>> 8u),
			static_cast<uint8_t>(out>> 0u)
		};
		uint8_t crc = calcCRC(out_datagram, 7);
		if ((crc != static_cast<uint8_t>(out)) || crc == 0 ) {
			CRCerror = true;
			out = 0;
		} else {
			break;
		}
	}

	return out>>8u;
}

uint8_t TMC2208Stepper::IFCNT() {
	return read(IFCNT_t::address);
}

void TMC2208Stepper::SLAVECONF(uint16_t input) {
    (void)input;
//	SLAVECONF_register.sr = input&0xF00;
//	write(SLAVECONF_register.address, SLAVECONF_register.sr);
}
uint16_t TMC2208Stepper::SLAVECONF() {
//	return SLAVECONF_register.sr;
    return 0;
}
void TMC2208Stepper::senddelay(uint8_t B) 	{
    (void)B;
//    SLAVECONF_register.senddelay = B;
//    write(SLAVECONF_register.address, SLAVECONF_register.sr);
}
uint8_t TMC2208Stepper::senddelay() 		{
//    return SLAVECONF_register.senddelay;
    return 0;
}

void TMC2208Stepper::OTP_PROG(uint16_t input) {
	write(OTP_PROG_t::address, input);
}

uint32_t TMC2208Stepper::OTP_READ() {
	return read(OTP_READ_t::address);
}

uint32_t TMC2208Stepper::IOIN() {
	return read(TMC2208_n::IOIN_t::address);
}
bool TMC2208Stepper::enn()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.enn;		}
bool TMC2208Stepper::ms1()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.ms1;		}
bool TMC2208Stepper::ms2()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.ms2;		}
bool TMC2208Stepper::diag()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.diag;		}
bool TMC2208Stepper::pdn_uart()		{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.pdn_uart;	}
bool TMC2208Stepper::step()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.step;		}
bool TMC2208Stepper::sel_a()		{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.sel_a;	}
bool TMC2208Stepper::dir()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.dir;		}
uint8_t TMC2208Stepper::version() 	{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.version;	}

uint16_t TMC2208Stepper::FACTORY_CONF() {
	return read(FACTORY_CONF_register.address);
}
void TMC2208Stepper::FACTORY_CONF(uint16_t input) {
	FACTORY_CONF_register.sr = input;
	write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr);
}
void TMC2208Stepper::fclktrim(uint8_t B){ FACTORY_CONF_register.fclktrim = B; write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr); }
void TMC2208Stepper::ottrim(uint8_t B)	{ FACTORY_CONF_register.ottrim = B; write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr); }
uint8_t TMC2208Stepper::fclktrim()		{ FACTORY_CONF_t r{0}; r.sr = FACTORY_CONF(); return r.fclktrim; }
uint8_t TMC2208Stepper::ottrim()		{ FACTORY_CONF_t r{0}; r.sr = FACTORY_CONF(); return r.ottrim; }

void TMC2208Stepper::VACTUAL(uint32_t input) {
	VACTUAL_register.sr = input;
	write(VACTUAL_register.address, VACTUAL_register.sr);
}
uint32_t TMC2208Stepper::VACTUAL() {
	return VACTUAL_register.sr;
}

uint32_t TMC2208Stepper::PWM_SCALE() {
	return read(TMC2208_n::PWM_SCALE_t::address);
}
uint8_t TMC2208Stepper::pwm_scale_sum() {
	TMC2208_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_sum;
}

int16_t TMC2208Stepper::pwm_scale_auto() {
	TMC2208_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_auto;
	// Not two's complement? 9nth bit determines sign
	/*
	uint32_t d = PWM_SCALE();
	int16_t response = (d>>PWM_SCALE_AUTO_bp)&0xFF;
	if (((d&PWM_SCALE_AUTO_bm) >> 24) & 0x1) return -response;
	else return response;
	*/
}

// R: PWM_AUTO
uint32_t TMC2208Stepper::PWM_AUTO() {
	return read(PWM_AUTO_t::address);
}
uint8_t TMC2208Stepper::pwm_ofs_auto()  {
    PWM_AUTO_t r{0}; r.sr = PWM_AUTO();
    return r.pwm_ofs_auto;
}
uint8_t TMC2208Stepper::pwm_grad_auto() {
    PWM_AUTO_t r{0};
    r.sr = PWM_AUTO();
    return r.pwm_grad_auto;
}

#define SET_REG(SETTING) CHOPCONF_register.SETTING = B; write(CHOPCONF_register.address, CHOPCONF_register.sr)
void TMC2208Stepper::CHOPCONF(uint32_t input) {
    CHOPCONF_register.sr = input;
    write(CHOPCONF_register.address, CHOPCONF_register.sr);
}
uint32_t TMC2208Stepper::CHOPCONF() {
    return read(CHOPCONF_register.address);
}

void TMC2208Stepper::toff	( uint8_t  B )	{ SET_REG(toff); 	}
void TMC2208Stepper::hstrt	( uint8_t  B )	{ SET_REG(hstrt); 	}
void TMC2208Stepper::hend	( uint8_t  B )	{ SET_REG(hend); 	}
void TMC2208Stepper::tbl	( uint8_t  B )	{ SET_REG(tbl); 	}
void TMC2208Stepper::vsense	( bool     B )	{ SET_REG(vsense); 	}
void TMC2208Stepper::mres	( uint8_t  B )	{ SET_REG(mres); 	}
void TMC2208Stepper::intpol	( bool     B )	{ SET_REG(intpol); 	}
void TMC2208Stepper::dedge	( bool     B )	{ SET_REG(dedge); 	}
void TMC2208Stepper::diss2g	( bool     B )	{ SET_REG(diss2g); 	}
void TMC2208Stepper::diss2vs( bool     B )	{ SET_REG(diss2vs); }

uint8_t TMC2208Stepper::toff()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.toff; 	}
uint8_t TMC2208Stepper::hstrt()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.hstrt; 	}
uint8_t TMC2208Stepper::hend()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.hend; 	}
uint8_t TMC2208Stepper::tbl()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.tbl;	 	}
bool 	TMC2208Stepper::vsense()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.vsense; 	}
uint8_t TMC2208Stepper::mres()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.mres; 	}
bool 	TMC2208Stepper::intpol()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.intpol; 	}
bool 	TMC2208Stepper::dedge()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.dedge; 	}
bool 	TMC2208Stepper::diss2g()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.diss2g; 	}
bool 	TMC2208Stepper::diss2vs()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.diss2vs;  }
#undef SET_REG


#define SET_REG(SETTING) GCONF_register.SETTING = B; write(GCONF_register.address, GCONF_register.sr)
uint32_t TMC2208Stepper::GCONF() {
    return read(GCONF_register.address);
}
void TMC2208Stepper::GCONF(uint32_t input) {
    GCONF_register.sr = input;
    write(GCONF_register.address, GCONF_register.sr);
}

void TMC2208Stepper::I_scale_analog(bool B)		{ SET_REG(i_scale_analog);	}
void TMC2208Stepper::internal_Rsense(bool B)	{ SET_REG(internal_rsense);	}
void TMC2208Stepper::en_spreadCycle(bool B)		{ SET_REG(en_spreadcycle);	}
void TMC2208Stepper::shaft(bool B) 				{ SET_REG(shaft);			}
void TMC2208Stepper::index_otpw(bool B)			{ SET_REG(index_otpw);		}
void TMC2208Stepper::index_step(bool B)			{ SET_REG(index_step);		}
void TMC2208Stepper::pdn_disable(bool B)		{ SET_REG(pdn_disable);		}
void TMC2208Stepper::mstep_reg_select(bool B)	{ SET_REG(mstep_reg_select);}
void TMC2208Stepper::multistep_filt(bool B)		{ SET_REG(multistep_filt);	}

bool TMC2208Stepper::I_scale_analog()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.i_scale_analog;		}
bool TMC2208Stepper::internal_Rsense()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.internal_rsense;	}
bool TMC2208Stepper::en_spreadCycle()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.en_spreadcycle;		}
bool TMC2208Stepper::shaft()			{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.shaft;				}
bool TMC2208Stepper::index_otpw()		{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.index_otpw;			}
bool TMC2208Stepper::index_step()		{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.index_step;			}
bool TMC2208Stepper::pdn_disable()		{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.pdn_disable;		}
bool TMC2208Stepper::mstep_reg_select()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.mstep_reg_select;	}
bool TMC2208Stepper::multistep_filt()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.multistep_filt;		}
#undef SET_REG


#define GET_REG(NS, SETTING) NS::DRV_STATUS_t r{0}; r.sr = DRV_STATUS(); return r.SETTING

uint32_t TMC2208Stepper::DRV_STATUS() {
    return read(TMC2208_n::DRV_STATUS_t::address);
}

bool 		TMC2208Stepper::otpw()		{ GET_REG(TMC2208_n, otpw); 		}
bool 		TMC2208Stepper::ot() 		{ GET_REG(TMC2208_n, ot); 	 		}
bool 		TMC2208Stepper::s2ga() 		{ GET_REG(TMC2208_n, s2ga); 		}
bool 		TMC2208Stepper::s2gb() 		{ GET_REG(TMC2208_n, s2gb); 		}
bool 		TMC2208Stepper::s2vsa() 	{ GET_REG(TMC2208_n, s2vsa);		}
bool 		TMC2208Stepper::s2vsb() 	{ GET_REG(TMC2208_n, s2vsb);		}
bool 		TMC2208Stepper::ola() 		{ GET_REG(TMC2208_n, ola);  		}
bool 		TMC2208Stepper::olb() 		{ GET_REG(TMC2208_n, olb);  		}
bool 		TMC2208Stepper::t120() 		{ GET_REG(TMC2208_n, t120); 		}
bool 		TMC2208Stepper::t143() 		{ GET_REG(TMC2208_n, t143); 		}
bool 		TMC2208Stepper::t150() 		{ GET_REG(TMC2208_n, t150); 		}
bool 		TMC2208Stepper::t157() 		{ GET_REG(TMC2208_n, t157); 		}
uint16_t 	TMC2208Stepper::cs_actual()	{ GET_REG(TMC2208_n, cs_actual);	}
bool 		TMC2208Stepper::stealth() 	{ GET_REG(TMC2208_n, stealth);		}
bool 		TMC2208Stepper::stst() 		{ GET_REG(TMC2208_n, stst); 		}
#undef GET_REG


#define SET_REG(SETTING) PWMCONF_register.SETTING = B; write(PWMCONF_register.address, PWMCONF_register.sr)
#define GET_REG(SETTING) return PWMCONF_register.SETTING
uint32_t TMC2208Stepper::PWMCONF() {
    return read(PWMCONF_register.address);
}
void TMC2208Stepper::PWMCONF(uint32_t input) {
    PWMCONF_register.sr = input;
    write(PWMCONF_register.address, PWMCONF_register.sr);
}

void TMC2208Stepper::pwm_ofs		( uint8_t B ) { PWMCONF_register.pwm_ofs = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_grad		( uint8_t B ) { PWMCONF_register.pwm_grad = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_freq		( uint8_t B ) { PWMCONF_register.pwm_freq = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_autoscale	( bool 	  B ) { PWMCONF_register.pwm_autoscale = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_autograd	( bool    B ) { PWMCONF_register.pwm_autograd = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::freewheel		( uint8_t B ) { PWMCONF_register.freewheel = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_reg		( uint8_t B ) { PWMCONF_register.pwm_reg = B; write(PWMCONF_register.address, PWMCONF_register.sr); }
void TMC2208Stepper::pwm_lim		( uint8_t B ) { PWMCONF_register.pwm_lim = B; write(PWMCONF_register.address, PWMCONF_register.sr); }

uint8_t TMC2208Stepper::pwm_ofs()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_ofs;		}
uint8_t TMC2208Stepper::pwm_grad()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_grad;		}
uint8_t TMC2208Stepper::pwm_freq()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_freq;		}
bool 	TMC2208Stepper::pwm_autoscale()	{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autoscale;	}
bool 	TMC2208Stepper::pwm_autograd()	{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_autograd;	}
uint8_t TMC2208Stepper::freewheel()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.freewheel;		}
uint8_t TMC2208Stepper::pwm_reg()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_reg;		}
uint8_t TMC2208Stepper::pwm_lim()		{ TMC2208_n::PWMCONF_t r{0}; r.sr = PWMCONF(); return r.pwm_lim;		}
#undef SET_REG
#undef GET_REG