#ifndef __BALENA_H_INCLUDED__
#define __BALENA_H_INCLUDED__

#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtcc.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_idac.h"
#include "em_chip.h"
#include "em_timer.h"
#include "em_i2c.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "hal-config.h"
#include "ustimer.h"
#include "tempdrv.h"
#include "rtcdriver.h"

#include "bsphalconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BGM111 Config */
#define EFR32BGM111 1
#define NUM_PINS 20
#define DAC_COUNT 6
#define DEVINFO_MODULEINFO_HFXOCALVAL_MASK  0x00080000UL
// Calibration value for HFXO CTUNE is at DEVINFO Offset 0x08
#define DEVINFO_MODULEINFO_CRYSTALOSCCALVAL  (*((uint16_t *) (uint32_t)(DEVINFO_BASE+0x8UL)))
// [15:9] : (LFXOTUNING) Calibration for LFXO TUNING
// [8:0]  : (HFXOCTUNE) Calibration for HFXO CTUNE
#define DEVINFO_HFXOCTUNE_MASK  0x01FFUL

/* Arduino Defines */
#define HIGH 1
#define LOW  0

/* balenaFin Pin Modes */
#define MODE_NONE        0
#define MODE_DIGITAL     1
#define MODE_ANALOG_IN   2
#define MODE_PWM		 3
#define MODE_I2C         4
#define MODE_SPI         5
#define MODE_ANALOG_OUT	 12

/* balenaFin Digital Pin Modes */
#define GPIO_INPUT_PULLUP gpioModeInputPull
#define GPIO_INPUT        gpioModeInput
#define GPIO_OUTPUT       gpioModePushPull
#define PWM_NONE 0x11111111UL
#define RESET 0x00000000UL

/* ADC */
#define adcFreq 13000000
#define PWM_FREQ 1000
#define TIMER_CHANNELS 4
#define TIMER_NONE 0x11111111UL

/* I2C */
#define CORE_FREQUENCY  14000000
#define RTC_MIN_TIMEOUT 32000
#define I2C_RXBUFFER_SIZE 10

typedef unsigned char byte;

/* Pin Struct */
struct PIN_MAP {
	GPIO_Port_TypeDef port;
	unsigned int pin;
	ADC_PosSel_TypeDef adc;
	IDAC_OutMode_TypeDef dac;
	unsigned short state;
	uint32_t pwm_0;
	uint32_t pwm_1;
	uint32_t pwm_2;
	uint32_t pwm_3;
};

/* Timer Index */
struct TIMER_INDEX {
	uint32_t pin;
	uint32_t route;
};

/* Init Functions */
void balenaInit();
void initMCU();
void initMCU_CLK();
void initGPIO();
void initTimer();
void initIDAC();
void initADC();
void initPWM();

/* Arduino Functions */

void delay(unsigned int n); // delay for n milliseconds
void pinMode(unsigned int pin_no, GPIO_Mode_TypeDef mode,unsigned int direction);
uint32_t millis();

void digitalWrite(unsigned int pin_no, unsigned int value);
unsigned int digitalRead(unsigned int pin_no);

void analogWrite(unsigned int pin_no, byte value);
uint32_t analogRead(unsigned int pin_no);

/* BalenaFin Functions */
extern PIN_MAP port_pin[NUM_PINS];
void reset();

/* Analog Functions */
void setADC(unsigned int pin_no, ADC_TypeDef * adc);
void setIDAC(unsigned int pin_no, IDAC_TypeDef * dac);
void resetPWM(byte pin);
bool setPWM(unsigned int pin_no, byte duty_cycle);

/* GPIO Functions */
void deviceMode(unsigned int pin_no, unsigned int mode);

/* I2C Functions */
void initI2C(void);
// TODO

/* SPI Functions */
// TODO

/* Math Functions */
long map(long x, long in_min, long in_max, long out_min, long out_max);

#ifdef __cplusplus
}
#endif

#endif
