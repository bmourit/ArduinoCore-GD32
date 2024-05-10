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

#ifndef UART_H
#define UART_H

#include "PinNames.h"
#include "PeripheralNames.h"

#include "gd32xxyy.h"
#include "PeripheralPins.h"
#include "pinmap.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UART_IRQ_PRIORITY
#define UART_IRQ_PRIORITY     1
#endif
#ifndef UART_IRQ_SUBPRIORITY
#define UART_IRQ_SUBPRIORITY  0
#endif

#define SERIAL_EVENT_TX_SHIFT (2)
#define SERIAL_EVENT_RX_SHIFT (8)
#define SERIAL_EVENT_TX_MASK (0x00FC)
#define SERIAL_EVENT_RX_MASK (0x3F00)
#define SERIAL_EVENT_ERROR (1 << 1)

/**
 * @defgroup SerialTXEvents Serial TX Events Macros
 *
 * @{
 */
#define SERIAL_EVENT_TX_COMPLETE (1 << (SERIAL_EVENT_TX_SHIFT + 0))
#define SERIAL_EVENT_TX_ALL      (SERIAL_EVENT_TX_COMPLETE)
/**@}*/

/**
 * @defgroup SerialRXEvents Serial RX Events Macros
 *
 * @{
 */
#define SERIAL_EVENT_RX_COMPLETE        (1 << (SERIAL_EVENT_RX_SHIFT + 0))
#define SERIAL_EVENT_RX_OVERRUN_ERROR   (1 << (SERIAL_EVENT_RX_SHIFT + 1))
#define SERIAL_EVENT_RX_FRAMING_ERROR   (1 << (SERIAL_EVENT_RX_SHIFT + 2))
#define SERIAL_EVENT_RX_PARITY_ERROR    (1 << (SERIAL_EVENT_RX_SHIFT + 3))
#define SERIAL_EVENT_RX_OVERFLOW        (1 << (SERIAL_EVENT_RX_SHIFT + 4))
#define SERIAL_EVENT_RX_CHARACTER_MATCH (1 << (SERIAL_EVENT_RX_SHIFT + 5))
#define SERIAL_EVENT_RX_ALL             (SERIAL_EVENT_RX_OVERFLOW | SERIAL_EVENT_RX_PARITY_ERROR | \
                     SERIAL_EVENT_RX_FRAMING_ERROR | SERIAL_EVENT_RX_OVERRUN_ERROR | \
                     SERIAL_EVENT_RX_COMPLETE | SERIAL_EVENT_RX_CHARACTER_MATCH)
/**@}*/

#define SERIAL_RESERVED_CHAR_MATCH (255)

#define USART_NO_ERROR            0x00U
#define USART_PARITY_ERROR        0x01U
#define USART_NOISE_ERROR         0x02U
#define USART_FRAME_ERROR         0x04U
#define USART_OVERRUN_ERROR       0x08U
#define USART_DMA_TRANSFER_ERROR  0x10U
//#define USART_NOISE_ERROR     0x20U

/* these are the same for all supported chip */
#define USART_RX_MODE             ((uint32_t)USART_CTL0_REN)
#define USART_TX_MODE             ((uint32_t)USART_CTL0_TEN)
#define USART_RXTX_MODE           ((uint32_t)(USART_CTL0_REN | USART_CTL0_TEN))

#define USART_TIMEOUT 1000

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

typedef struct {
  uint32_t baudrate;
  uint32_t databits;
  uint32_t stopbits;
  uint32_t parity;
  uint32_t mode;
  uint32_t hw_flow_control;
  uint32_t oversample;
} uart_params_t;

typedef struct {
  uint32_t instance;
  uart_params_t params;
  uint8_t *tx_buffer_ptr;
  uint16_t tx_size;
  __IO uint16_t tx_count;
  uint8_t *rx_buffer_ptr;
  uint16_t rx_size;
  __IO uint16_t rx_count;
  operation_state_enum global_state;
  operation_state_enum rx_state;
  __IO uint32_t error_code;
} SPL_UartHandle_t;

struct serial_s {
  UARTName uart;
  SPL_UartHandle_t handle;
  int (*tx_callback)(serial_t *obj);
  void (*rx_callback)(serial_t *obj);
  PinName pin_tx;
  PinName pin_rx;
  uint8_t index;
  uint8_t rvalue;
  uint8_t *rx_buff;
  uint8_t *tx_buff;
  uint16_t rx_tail;
  uint16_t tx_head;
  volatile uint16_t rx_head;
  volatile uint16_t tx_tail;
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
  UART_NUM
} uart_index_t;

void serial_init(serial_t *obj, uint32_t bauderate, uint32_t databits, uint32_t parity, uint32_t stopbits);
void serial_free(serial_t *obj);

size_t serial_write(serial_t *obj, uint8_t data, uint16_t size);
int serial_getc(serial_t *obj, unsigned char *c);
void uart_attach_tx_callback(serial_t *obj, int(*callback)(serial_t *));
void uart_attach_rx_callback(serial_t *obj, void(*callback)(serial_t *));

uint8_t serial_tx_active(serial_t *obj);
uint8_t serial_rx_active(serial_t *obj);

void serial_enable_tx(serial_t *obj);
void serial_enable_rx(serial_t *obj);

void serial_debug_init(void);
size_t serial_debug_write(uint8_t *data, uint32_t size);

/* TX transfer (blocking) */
gd_status_enum serial_transmit(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size, uint32_t timeout);
/* RX transfer (blocking) */
gd_status_enum serial_receive(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size, uint32_t timeout);
/* TX transfer (non-blocking) */
gd_status_enum IT_serial_transmit(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size);
/* RX transfer (non-blocking) */
gd_status_enum IT_serial_receive(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size);


/* extended and helper functions */
gd_status_enum IT_transmit(SPL_UartHandle_t *uart_handle);
gd_status_enum IT_receive(SPL_UartHandle_t *uart_handle);
gd_status_enum serial_wait_flag_timeout(SPL_UartHandle_t *uart_handle,
    uint32_t flag, FlagStatus status, uint32_t timeout);

void UART_TX_TCCallback(SPL_UartHandle_t *uart_handle);
void UART_ErrorCallback(SPL_UartHandle_t *uart_handle);

void UART_RX_TCCallback(SPL_UartHandle_t *uart_handle);
void UART_TX_TCCallback(SPL_UartHandle_t *uart_handle);

void serial_rx_transfer_end(SPL_UartHandle_t *uart_handle);
void IT_serial_transmit_end(SPL_UartHandle_t *uart_handle);

void UART_IRQHandler(SPL_UartHandle_t *uart_handle);

void serial_half_duplex_enable_tx(SPL_UartHandle_t *uart_handle);
void serial_half_duplex_enable_rx(SPL_UartHandle_t *uart_handle);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
