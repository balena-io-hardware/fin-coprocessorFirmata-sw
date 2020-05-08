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
#include "balenaconfig.h"
#include "ustimer.h"
#include "tempdrv.h"
#include "rtcdriver.h"
#include "version.h"
#include <cstddef>
#include <string>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* BGM111 Config */
#define EFR32BGM111 1
#define NUM_PINS 20
#define DAC_COUNT 6
#define DEVINFO_MODULEINFO_HFXOCALVAL_MASK 0x00080000UL
// Calibration value for HFXO CTUNE is at DEVINFO Offset 0x08
#define DEVINFO_MODULEINFO_CRYSTALOSCCALVAL (*((uint16_t *)(uint32_t)(DEVINFO_BASE + 0x8UL)))
// [15:9] : (LFXOTUNING) Calibration for LFXO TUNING
// [8:0]  : (HFXOCTUNE) Calibration for HFXO CTUNE
#define DEVINFO_HFXOCTUNE_MASK 0x01FFUL

/* Arduino Defines */
#define HIGH 1
#define LOW 0

/* balenaFin Pin Modes */
#define MODE_NONE 0xFFFFUL

#define MODE_DIGITAL_INPUT 0
#define MODE_DIGITAL 1
#define MODE_ANALOG_IN 2
#define MODE_PWM 3
#define MODE_I2C 4
#define MODE_SPI 5
#define MODE_ANALOG_OUT 12
#define SLEEP_PIN 16

/* balenaFin Digital Pin Modes */
#define GPIO_INPUT_PULLUP gpioModeInputPull
#define GPIO_INPUT gpioModeInput
#define GPIO_OUTPUT gpioModePushPull
#define PWM_NONE 0x11111111UL
#define RESET 0x00000000UL

/* ADC */
#define adcFreq 13000000
#define PWM_FREQ 1000
#define TIMER_CHANNELS 4
#define TIMER_NONE 0x11111111UL

/* I2C */
#define CORE_FREQUENCY 14000000
#define RTC_MIN_TIMEOUT 32000
#define I2C_RXBUFFER_SIZE 10
#define I2C_TXBUFFER_SIZE 10
#define CMD_ARRAY_SIZE 1
#define DATA_ARRAY_SIZE 10
#define I2C_ERR 0xFF

/* SYSEX Balena */
#define BALENA_ERR 0xFF
#define BALENA_SUCCESS 0xFE

typedef unsigned char byte;

extern RTCDRV_TimerID_t id;

/* Pin Struct */
struct PIN_MAP
{
	GPIO_Port_TypeDef port;
	unsigned int pin;
	unsigned int adc;
	unsigned int idac;
	unsigned short state;
	uint32_t pwm_0;
	uint32_t pwm_1;
	uint32_t pwm_2;
	uint32_t pwm_3;
};

struct IDAC_LOOKUP
{
	IDAC_OutMode_TypeDef idac;
};

struct IDAC_RANGE_LOOKUP
{
	IDAC_Range_TypeDef idac_range;
};

struct ADC_LOOKUP
{
	ADC_PosSel_TypeDef adc;
};

/* Timer Index */
struct TIMER_INDEX
{
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
void pinMode(unsigned int pin_no, GPIO_Mode_TypeDef mode, unsigned int direction);
uint32_t millis();

/* GPIO Functions */
void digitalWrite(unsigned int pin_no, unsigned int value);
unsigned int digitalRead(unsigned int pin_no);

void analogWrite(unsigned int pin_no, byte value);
uint32_t analogRead(unsigned int pin_no);

/* BalenaFin Functions */
extern PIN_MAP port_pin[NUM_PINS];
void reset();

/* Analog Functions */
int setADC(unsigned int pin_no, ADC_TypeDef *adc);
int setIDAC(unsigned int pin_no, byte range);
int writeIDAC(unsigned int pin_no, byte step);
void resetIDAC();
void resetPWM(byte pin);
int setPWM(unsigned int pin_no, byte duty_cycle);

/* Enable/Disable VCOM */
void serialMode(bool enable);

/* RTC Functions */
void triggerEvent(uint32_t timeout, RTCDRV_Callback_t callback);

/* I2C Functions */
void initI2C(byte mode);
void deinitI2C();
int transferI2C(uint16_t device_addr, uint8_t cmd_array[], uint8_t data_array[], uint16_t cmd_len, uint16_t data_len, uint8_t flag);

/* SPI Functions */
// TODO

/* Math Functions */
long map(long x, long in_min, long in_max, long out_min, long out_max);

/* Helper Functions */
char *insertInt(std::string str, int i);

#ifdef __cplusplus
}
#endif

#endif
