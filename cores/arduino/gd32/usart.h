/*
	Copyright (c) 2024 GD32CommunityCores
	Copyright (c) 2020 GigaDevice Semiconductor Inc.
  	Copyright (c) 2016-2021 STMicroelectronics

	Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.
	3. Neither the name of the copyright holder nor the names of its contributors
	   may be used to endorse or promote products derived from this software without
	   specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef USART_H
#define USART_H

/* Includes */
#include "gd32xxyy.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "pinmap.h"
#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UART_IRQ_PRIORITY
#define UART_IRQ_PRIORITY     1
#endif
#ifndef UART_IRQ_SUBPRIORITY
#define UART_IRQ_SUBPRIORITY  0
#endif

#define USART_NO_ERROR            0x00U
#define USART_PARITY_ERROR        0x01U
#define USART_NOISE_ERROR         0x02U
#define USART_FRAME_ERROR         0x04U
#define USART_OVERRUN_ERROR       0x08U
#define USART_DMA_TRANSFER_ERROR  0x10U
#define USART_NOISE_ERROR			    0x20U

#define USART_TIMEOUT	1000

/**
 *  gd_{status,operation_state}_enum cribbed from previous library
 * and put here because they don’t appear in the GD firmware library
 * upstream, and appear to have been added after-the-fact in
 * gd32f30x.h by some porter. -bjc (2021-Aug-20)
 */
typedef enum {
	GD_OK       = 0x00U,
	GD_ERROR    = 0x01U,
	GD_BUSY     = 0x02U,
	GD_TIMEOUT  = 0x03U
} gd_status_enum;

typedef enum {
	OP_STATE_RESET             = 0x00U,
	OP_STATE_READY             = 0x01U,
	OP_STATE_BUSY              = 0x02U,
	OP_STATE_TIMEOUT           = 0x03U,
	OP_STATE_ERROR             = 0x04U,
	OP_STATE_ABORT             = 0x05U,
	OP_STATE_LISTEN            = 0x06U,
	OP_STATE_BUSY_TX           = 0x21U, /* (OP_STATE_BUSY << 4) + 1 */
	OP_STATE_BUSY_RX           = 0x22U, /* (OP_STATE_BUSY << 4) + 2 */
	OP_STATE_BUSY_TX_LISTEN    = 0x61U, /* (OP_STATE_LISTEN << 4) + 1 */
	OP_STATE_BUSY_RX_LISTEN    = 0x62U, /* (OP_STATE_LISTEN << 4) + 2 */
	OP_STATE_BUTT
} operation_state_enum;

typedef enum {
	ParityNone = 0,
	ParityOdd = 1,
	ParityEven = 2,
	ParityForced1 = 3,
	ParityForced0 = 4
} SerialParity;

typedef struct serial_s serial_t;

struct serial_s {
	/* basic information */
	UARTName uart;
	void (*tx_callback)(serial_t *obj);
	void (*rx_callback)(serial_t *obj);
	PinName pin_tx;
	PinName pin_rx;
	uint8_t index;
	/* config info */
	uint32_t baudrate;
	uint32_t databits;
	uint32_t stopbits;
	SerialParity parity;
	bool halfduplex;
	/* operating params */
	uint8_t receiver;
	uint8_t *tx_buffer_ptr;
	uint8_t *rx_buffer_ptr;
	/* used in HardwareSerial */
	uint16_t tx_count;
	uint16_t rx_count;
	uint8_t *rx_buff;
	uint8_t *tx_buff;
	uint16_t rx_tail;
	uint16_t tx_head;
	volatile uint16_t rx_head;
	volatile uint16_t tx_tail;
	uint32_t error_code;
	operation_state_enum tx_state;
	operation_state_enum rx_state;
	operation_state_enum uart_state;
	uint16_t tx_size;
};

typedef enum {
#if defined(USART0)
	UART0_INDEX,
#endif
#if defined(USART1)
	UART1_INDEX,
#endif
#if defined(USART2)
	UART2_INDEX,
#endif
#if defined(UART3) || defined(USART3)
	UART3_INDEX,
#endif
#if defined(UART4) || defined(USART4)
	UART4_INDEX,
#endif
	NUM_UART
} uart_index_t;

/* Initialize the serial peripheral. It sets the default parameters for serial peripheral, and configures its specifieds pins. */
void serial_init(serial_t *obj, uint32_t baudrate, uint32_t databits, SerialParity parity, uint32_t stopbits);
void serial_enable_tx(serial_t *obj);
void serial_enable_rx(serial_t *obj);
void uart_deinit(serial_t *obj);
void serial_debug_init(void);
size_t serial_debug_write(uint8_t *data, uint32_t size);
/* Release the serial peripheral, not currently invoked. It requires further resource management. */
void serial_free(serial_t *obj);
/* Configure the baud rate */
void serial_baud(serial_t *obj, int baudrate);
/* Configure the format. Set the number of bits, parity and the number of stop bits. */
void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits);
/* Get character */
int serial_getc(serial_t *obj, unsigned char *c);
/* Send a character. This is a blocking call, waiting for a peripheral to be available for writing. */
size_t uart_write(setial_t *obj, uint8_t data, uint16_t size);
//void serial_putc(serial_t *obj, int c);
/* Check if the serial peripheral is readable. */
int serial_readable(serial_t *obj);
/* Check if the serial peripheral is writable. */
int serial_writable(serial_t *obj);
/* Clear the serial peripheral. */
void serial_clear(serial_t *obj);
/* Attempts to determine if the serial peripheral is already in use for TX. */
uint8_t serial_tx_active(serial_t *obj);
/* Attempts to determine if the serial peripheral is already in use for RX. */
uint8_t serial_rx_active(serial_t *obj);
/* Attach UART transmit callback */
void uart_attach_tx_callback(serial_t *obj, void(*callback)(serial_t *));
/* Attach UART receive callback */
void uart_attach_rx_callback(serial_t *obj, void(*callback)(serial_t *));
/* Begin asynchronous TX transfer */
gd_status_enum serial_transmit(serial_t *obj, const uint8_t *txData, size_t txsize);
/* Begin asynchronous RX transfer (enable interrupt for data collecting) */
gd_status_enum serial_receive(serial_t *obj, const uint8_t *rxData, size_t rxsize);
/* non-blocking */
gd_status_enum serial_transmit_nb(serial_t *obj, const uint8_t *txData, size_t txSize);
/* non-blocking */
gd_status_enum serial_receive_nb(serial_t *obj, uint8_t *rxData, size_t rxSize);
/* tx transfer complete callback */
void uart_rx_transfercomplete(serial_t *obj);
/* rx transfer complete callback */
void uart_tx_transfercomplete(serial_t *obj);
/* error callback */
void uart_error_callback(struct serial_s *obj_s);
/* IRQ Handler */
void UART_IRQHandler(struct serial_s *obj_s)

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
