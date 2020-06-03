#ifndef __SERIAL_H__
#define __SERIAL_H__

// #include "hal-config.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "balena.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__cplusplus) && !defined(ARDUINO)
  #include <cstddef>
  #include <cstdint>
#else
  #include <stddef.h>
  #include <stdint.h>
#endif

#define FIN_LOC USART_ROUTELOC0_RXLOC_LOC25|USART_ROUTELOC0_TXLOC_LOC27
#define FIN_PIN_RX gpioPortF,2
#define FIN_PIN_TX gpioPortF,3

#define DEV_LOC USART_ROUTELOC0_RXLOC_LOC0|USART_ROUTELOC0_TXLOC_LOC0
#define DEV_PIN_RX gpioPortA,1
#define DEV_PIN_TX gpioPortA,0

#define EXT_LOC USART_ROUTELOC0_RXLOC_LOC2|USART_ROUTELOC0_TXLOC_LOC2
#define EXT_PIN_RX gpioPortA,3
#define EXT_PIN_TX gpioPortA,2

// #define LOC DEV_LOC
// #define RX DEV_PIN_RX
// #define TX DEV_PIN_TX

#define EMDRV_UARTDRV_MAX_CONCURRENT_BUFS 12
#define BUFFERSIZE 256
#define RESET 0x00000000UL

extern volatile uint32_t rx_data_ready;
extern volatile uint32_t rx_data_available;
extern volatile uint32_t tx_data_available;

typedef unsigned char byte;


class SerialClass{
	private:
		USART_InitAsync_TypeDef uartInit =
		{                                                                                                  \
			usartDisable,          /* Enable RX/TX when initialization is complete. */                    \
			0,                     /* Use current configured reference clock for configuring baud rate. */ \
			115200,                /* 115200 bits/s. */                                                    \
			usartOVS16,            /* 16x oversampling. */                                                 \
			usartDatabits8,        /* 8 data bits. */                                                      \
			usartNoParity,         /* No parity. */                                                        \
			usartStopbits1,        /* 1 stop bit. */                                                       \
			false,                 /* Do not disable majority vote. */                                     \
			false,                 /* Not USART PRS input mode. */                                         \
			0,                     /* PRS channel 0. */                                                    \
			false,                 /* Auto CS functionality enable/disable switch */                       \
			0,                     /* Auto CS Hold cycles */                                               \
			0,                     /* Auto CS Setup cycles */                                              \
			usartHwFlowControlNone /* No HW flow control */                                                \
		 };

	public:
		SerialClass();
		SerialClass(long baudrate);
		int available(void);
		int availableForWrite(void);
		void begin(long baudrate);
		void end();

		int peek(void);
		void flush(void);

		int baudRate(void);
		size_t getRxBufferSize();

		int read(void);
		uint32_t readBytes(uint8_t * dataPtr, uint32_t dataLen);
		size_t readBytesUntil();
		size_t readString();
		size_t readStringUntil();

		void write(uint8_t ch);
		void write(uint8_t * dataPtr, uint32_t dataLen);

		void setDebugOutput(bool);

		bool isTxEnabled(void);
		bool isRxEnabled(void);
		bool hasOverrun(void);
		bool hasRxError(void);

		void find();
		void findUntil();

		void setTimeout();
		size_t write();
		void serialEvent();

		void parseFloat();
		void parseInt();
};

#ifdef __cplusplus
}
#endif
#endif
