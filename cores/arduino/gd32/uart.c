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
#include "gd_debug.h"
#include "uart.h"
#include "Arduino.h"
#include "gd32F30x_remap.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(USART_DATA)
#define GD32_USART_TX_DATA USART_DATA
#define GD32_USART_RX_DATA USART_DATA
#if defined(GD32F10x)
#define GD32_USART_STAT    USART_STAT
#else
#define GD32_USART_STAT    USART_STAT0
#endif
#elif defined(USART_RDATA) && defined(USART_TDATA)
#define GD32_USART_TX_DATA USART_TDATA
#define GD32_USART_RX_DATA USART_RDATA
#define GD32_USART_STAT    USART_STAT
#else
#error "We don't understand this USART peripheral."
#endif

#if !defined(UART_DEBUG)
#if defined(PIN_SERIAL_TX)
#define UART_DEBUG          pinmap_peripheral(DIGITAL_TO_PINNAME(PIN_SERIAL_TX), PinMap_UART_TX)
#define PINNAME_TX_DEBUG    DIGITAL_TO_PINNAME(PIN_SERIAL_TX)
#else
#define UART_DEBUG          NP
#define PINNAME_TX_DEBUG    NC
#endif
#endif

#if !defined(UART_DEBUG_BAUDRATE)
#define UART_DEBUG_BAUDRATE 9600
#endif

static rcu_periph_enum usart_clk[UART_NUM] = {
  RCU_USART0,
  RCU_USART1,
  RCU_USART2,
#ifdef UART3
  RCU_UART3,
#elif USART3
  RCU_USART3,
#endif
#ifdef UART4
  RCU_UART4
#elif USART4
  RCU_USART4
#endif
};

static IRQn_Type usart_irq_n[UART_NUM] = {
  USART0_IRQn,
  USART1_IRQn,
  USART2_IRQn,
#ifdef UART3
  UART3_IRQn,
#elif USART3
  USART3_IRQn,
#endif
#ifdef UART4
  UART4_IRQn
#elif USART4
  USART4_IRQn
#endif
};

static SPL_UartHandle_t *uart_handlers[UART_NUM] = {NULL};

static serial_t serial_debug = { .uart = NP, .index = UART_NUM };

serial_t *get_serial_object(SPL_UartHandle_t *uart_handle)
{
	struct serial_s *obj_s = (struct serial_s *)((char *)uart_handle - offsetof(struct serial_s, handle));
	serial_t *obj = (serial_t *)((char *)obj_s - offsetof(serial_t, uart));

	return (obj);
}

static gd_status_enum usart_half_duplex_init(SPL_UartHandle_t *uart_handle)
{
	if (uart_handle == NULL) {
		return GD_ERROR;
	}

	uart_handle->global_state = OP_STATE_BUSY;

	usart_disable(uart_handle->instance);
	usart_word_length_set(uart_handle->instance, uart_handle->params.databits);
	usart_stop_bit_set(uart_handle->instance, uart_handle->params.stopbits);
	usart_parity_config(uart_handle->instance, uart_handle->params.parity);

	switch (uart_handle->params.mode) {
		case USART_RXTX_MODE:
	 		usart_receive_config(uart_handle->instance, USART_RECEIVE_ENABLE);
	 		usart_transmit_config(uart_handle->instance, USART_TRANSMIT_ENABLE);
	 		break;
		case USART_RX_MODE:
	 		usart_receive_config(uart_handle->instance, USART_RECEIVE_ENABLE);
	 		break;
		case USART_TX_MODE:
	 		usart_transmit_config(uart_handle->instance, USART_TRANSMIT_ENABLE);
	 		break;
	 	default:
	 		usart_receive_config(uart_handle->instance, USART_RECEIVE_DISABLE);
	 		usart_transmit_config(uart_handle->instance, USART_RECEIVE_DISABLE);
	}
	usart_baudrate_set(uart_handle->instance, uart_handle->params.baudrate);

	usart_lin_mode_disable(uart_handle->instance);
	usart_synchronous_clock_disable(uart_handle->instance);
	usart_smartcard_mode_disable(uart_handle->instance);
	usart_irda_mode_disable(uart_handle->instance);
	usart_halfduplex_enable(uart_handle->instance);

	usart_enable(uart_handle->instance);

	uart_handle->error_code = USART_NO_ERROR;
	uart_handle->global_state = OP_STATE_READY;
	uart_handle->rx_state = OP_STATE_READY;

	return GD_OK;
}

/**
 * Initialize the USART peripheral.
 *
 * @param obj_s The serial object
 */
static gd_status_enum usart_init(SPL_UartHandle_t *uart_handle)
{
	if (uart_handle == NULL) {
		return GD_ERROR;
	}

	uart_handle->global_state = OP_STATE_BUSY;

	usart_disable(uart_handle->instance);
	usart_word_length_set(uart_handle->instance, uart_handle->params.databits);
	usart_stop_bit_set(uart_handle->instance, uart_handle->params.stopbits);
	usart_parity_config(uart_handle->instance, uart_handle->params.parity);

	switch (uart_handle->params.mode) {
		case USART_RXTX_MODE:
	 		usart_receive_config(uart_handle->instance, USART_RECEIVE_ENABLE);
	 		usart_transmit_config(uart_handle->instance, USART_TRANSMIT_ENABLE);
	 		break;
		case USART_RX_MODE:
	 		usart_receive_config(uart_handle->instance, USART_RECEIVE_ENABLE);
	 		break;
		case USART_TX_MODE:
	 		usart_transmit_config(uart_handle->instance, USART_TRANSMIT_ENABLE);
	 		break;
	 	default:
	 		usart_receive_config(uart_handle->instance, USART_RECEIVE_DISABLE);
	 		usart_transmit_config(uart_handle->instance, USART_RECEIVE_DISABLE);
	}
	usart_baudrate_set(uart_handle->instance, uart_handle->params.baudrate);

	usart_lin_mode_disable(uart_handle->instance);
	usart_synchronous_clock_disable(uart_handle->instance);
	usart_smartcard_mode_disable(uart_handle->instance);
	usart_halfduplex_disable(uart_handle->instance);
	usart_irda_mode_disable(uart_handle->instance);

	usart_enable(uart_handle->instance);

	uart_handle->error_code = USART_NO_ERROR;
	uart_handle->global_state = OP_STATE_READY;
	uart_handle->rx_state = OP_STATE_READY;

	return GD_OK;
}

/**
 * Initialize the serial peripheral. It sets the default parameters for serial
 * peripheral, and configures the specified pins.
 *
 * @param obj The serial object
 * @param tx  The TX pin name
 * @param rx  The RX pin name
 */
void serial_init(serial_t *obj, uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits)
{
  if (obj == NULL) {
    return;
  }

	SPL_UartHandle_t *uart_handle = &(obj->handle);

	uint32_t uart_tx = pinmap_peripheral(obj->pin_tx, PinMap_UART_TX);
	uint32_t uart_rx = pinmap_peripheral(obj->pin_rx, PinMap_UART_RX);

	if (uart_tx == NP) {
		gd_debug("ERROR: [U(S)ART] TX pin has no peripheral!\n");
		return;
	}

	if ((obj->pin_rx != NC) && (uart_rx == NP)) {
		gd_debug("ERROR: [U(S)ART] RX pin has no peripheral!\n");
		return;
	}

	obj->uart = (UARTName)pinmap_merge(uart_tx, uart_rx);

  if (obj->uart == NP) {
    gd_debug("ERROR: [U(S)ART] RX and TX pins and peripherals mismatch!\n");
    return;
  }

	/* set uart index */
	switch (obj->uart) {
		case UART_0:
			obj->index = UART0_INDEX;
			break;
		case UART_1:
			obj->index = UART1_INDEX;
			break;
		case UART_2:
			obj->index = UART2_INDEX;
			break;
#if (defined(UART3) || defined(USART3))
		case UART_3:
			obj->index = UART3_INDEX;
			break;
#endif
#if (defined(UART4) || defined(USART4))
		case UART_4:
			obj->index = UART4_INDEX;
			break;
#endif
	}

	/* reset and enable UART peripheral clock */
  usart_deinit(obj->uart);
	rcu_periph_clock_enable(usart_clk[obj->index]);

	/* configure the pins */
	pinmap_pinout(obj->pin_tx, PinMap_UART_TX);
	if (uart_rx != NP) {
  	pinmap_pinout(obj->pin_rx, PinMap_UART_RX);
  }

  uart_handlers[obj->index] = uart_handle;
  uart_handle->instance = obj->uart;
	uart_handle->params.baudrate = baudrate;
	uart_handle->params.databits = databits;
	uart_handle->params.stopbits = stopbits;
	uart_handle->params.parity = parity;
	uart_handle->params.mode = USART_RXTX_MODE;
	uart_handle->params.hw_flow_control = 0x0U;
	uart_handle->params.oversample = 0x0U;

	if (uart_rx == NP) {
		if (usart_half_duplex_init(uart_handle) != GD_OK) {
			return;
		}
	} else if (usart_init(uart_handle) != GD_OK) {
		return;
	}
}

static void serial_deinit(SPL_UartHandle_t *uart_handle)
{
	uart_handle->global_state = OP_STATE_BUSY;

  usart_disable(uart_handle->instance);

  uart_handle->error_code = USART_NO_ERROR;
  uart_handle->global_state = OP_STATE_RESET;
  uart_handle->rx_state = OP_STATE_RESET;
}

/*!
 \brief  frees (deinitializes) the uart
 \param  obj: pointer to serial_t structure
 \retval none
*/
void serial_free(serial_t *obj)
{
	UARTName uartReg = 0U;

	switch (obj->index) {
	case UART0_INDEX:
		uartReg = UART_0;
		break;
	case UART1_INDEX:
		uartReg = UART_1;
		break;
	case UART2_INDEX:
		uartReg = UART_2;
		break;
#if (defined(UART3) || defined(USART3))
	case UART3_INDEX:
		uartReg = UART_3;
		break;
#endif
#if (defined(UART4) || defined(USART4))
	case UART4_INDEX:
		uartReg = UART_4;
		break;
#endif
	}

	/* reset uart and disable clock */
	if (uartReg != 0U) {
		usart_deinit(uartReg);
	}

	rcu_periph_clock_disable(usart_clk[obj->index]);
	serial_deinit(uart_handlers[obj->index]);

	/* free the uart debug for init */
	if (serial_debug.index == obj->index) {
		serial_debug.index = UART_NUM;
	}
}

/*!
  \brief  write the data on the uart
  \param  obj : pointer to serial_t structure
  \param  data : byte to write
  \param  size : number of data to write
  \retval returns the number of bytes written
*/
size_t serial_write(serial_t *obj, uint8_t data, uint16_t size)
{
	if (serial_transmit(uart_handlers[obj->index], &data, size, USART_TIMEOUT) == GD_OK) {
		return size;
	} else {
		return 0;
	}
}

void serial_debug_init(void)
{
	if (UART_DEBUG != NP) {
		serial_debug.pin_rx = pinmap_pin(UART_DEBUG, PinMap_UART_RX);
#if defined(PINNAME_TX_DEBUG)
		serial_debug.pin_tx = PINNAME_TX_DEBUG;
#else
		serial_debug.pin_tx = pinmap_pin(UART_DEBUG, PinMap_UART_TX);
#endif
		serial_init(&serial_debug, UART_DEBUG_BAUDRATE, USART_WL_8BIT, USART_PM_NONE, USART_STB_1BIT);
	}
}

size_t serial_debug_write(uint8_t *data, uint32_t size)
{
	if (UART_DEBUG == NP) {
		return 0;
	}

	IRQn_Type irq = 0;

	if (serial_debug.index >= UART_NUM) {
		for (serial_debug.index = 1; serial_debug.index < UART_NUM; serial_debug.index++) {
			if (uart_handlers[serial_debug.index] != NULL) {
				if (UART_DEBUG == uart_handlers[serial_debug.index]->instance) {
					break;
				}
			}
		}

		if (serial_debug.index >= UART_NUM) {
			serial_debug_init();
			if (serial_debug.index >= UART_NUM) {
				return 0;
			}
		} else {
			serial_t *obj = get_serial_object(uart_handlers[serial_debug.index]);
			if (obj) {
				irq = usart_irq_n[obj->index];
			}
		}
	}
	NVIC_DisableIRQ(irq);

	while (serial_transmit(uart_handlers[serial_debug.index], data, size, USART_TIMEOUT) != GD_OK) {
		for (int i = 0; i < USART_TIMEOUT; i++);
		size = 0;
		break;
	}
	NVIC_EnableIRQ(irq);

	return size;
}

/*!
 \brief   reads a rx byte from uart
 \param   obj: serial object pointer
 \retval  returns the last character received
*/
int serial_getc(serial_t *obj, unsigned char *c)
{
  if (obj == NULL) {
  	return -1;
  }

  if (serial_rx_active(obj)) {
  	return -1;
  }

  *c = (unsigned char)(obj->rvalue);
  IT_serial_receive(uart_handlers[obj->index], &(obj->rvalue), 1);

	return 0;
}

/**
 * Tries to determine if the serial is already being used for tx
 * @param obj serial object
 * @retval returns 1 if rx transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
	return ((uart_handlers[obj->index]->global_state == OP_STATE_BUSY_TX) ? 1 : 0);
}

/**
 * Tries to determine if the serial is already being used for rx
 * @param obj serial object
 * @retval returns 1 if rx transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
	return ((uart_handlers[obj->index]->rx_state == OP_STATE_BUSY_RX) ? 1 : 0);
}

/**
 * Attach UART transmit callback
 *
 * @param obj The serial object
 * @param callback The transmit callback
 */
void uart_attach_tx_callback(serial_t *obj, int(*callback)(serial_t *))
{
	if (obj == NULL) {
		return;
	}

	IRQn_Type irq = usart_irq_n[obj->index];
	obj->tx_callback = callback;

	NVIC_DisableIRQ(irq);
	IT_serial_transmit(uart_handlers[obj->index], &obj->tx_buff[obj->tx_tail], 1);
	nvic_irq_enable(irq, UART_IRQ_PRIORITY, UART_IRQ_SUBPRIORITY);
}

/**
 * Attach UART receive callback
 *
 * @param obj The serial object
 * @param callback The transmit callback
 */
void uart_attach_rx_callback(serial_t *obj, void(*callback)(serial_t *))
{
	if (obj == NULL) {
		return;
	}

	/* Exit if a reception is already on-going */
	if (serial_rx_active(obj)) {
		return;
	}

	IRQn_Type irq = usart_irq_n[obj->index];
	obj->rx_callback = callback;

	NVIC_DisableIRQ(irq);
	IT_serial_receive(uart_handlers[obj->index], &(obj->rvalue), 1);
	nvic_irq_enable(irq, UART_IRQ_PRIORITY, UART_IRQ_SUBPRIORITY);
}

void serial_enable_tx(serial_t *obj)
{
	if (obj != NULL && obj->pin_tx == NC) {
		serial_half_duplex_enable_tx(uart_handlers[obj->index]);
	}
}

void serial_enable_rx(serial_t *obj)
{
	if (obj != NULL && obj->pin_rx == NC) {
		serial_half_duplex_enable_rx(uart_handlers[obj->index]);
	}
}

/**
 *  Begin TX transfer (blocking) 
 */
gd_status_enum serial_transmit(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size, uint32_t timeout)
{
	uint16_t *tmp;
	if (uart_handle->global_state == OP_STATE_READY) {
		if ((pData == NULL) || (size == 0U)) {
			return GD_ERROR;
		}

		uart_handle->error_code = USART_NO_ERROR;
		uart_handle->global_state = OP_STATE_BUSY_TX;
		uart_handle->tx_size = size;
		uart_handle->tx_count = size;

		while (uart_handle->tx_count > 0U) {
			uart_handle->tx_count--;
			if (uart_handle->params.databits == USART_WL_9BIT) {
				if (serial_wait_flag_timeout(uart_handle, USART_FLAG_TBE, RESET, timeout) != GD_OK) {
					return GD_TIMEOUT;
				}
				tmp = (uint16_t *)pData;
				//usart_data_trasnmit(uart_handle->instance, *tmp);
				USART_DATA(uart_handle->instance) = USART_DATA_DATA & (uint32_t)*tmp;
				if (uart_handle->params.parity == USART_PM_NONE) {
					pData += 2U;
				} else {
					pData += 1U;
				}
			} else {
				if (serial_wait_flag_timeout(uart_handle, USART_FLAG_TBE, RESET, timeout) != GD_OK) {
					return GD_TIMEOUT;
				}
				//usart_data_transmit(uart_handle->instance, *pData++);
				USART_DATA(uart_handle->instance) = USART_DATA_DATA & (uint32_t)*pData++;
			}
		}

		if (serial_wait_flag_timeout(uart_handle, USART_FLAG_TC, RESET, timeout) != GD_OK) {
			return GD_TIMEOUT;
		}
		uart_handle->global_state = OP_STATE_READY;

		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

/**
 *  Begin TX transfer (non-blocking) 
 */
gd_status_enum IT_serial_transmit(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size)
{
	if (uart_handle->global_state == OP_STATE_READY) {
		if ((pData == NULL) || (size == 0U)) {
			return GD_ERROR;
		}

		uart_handle->tx_buffer_ptr = pData;
		uart_handle->tx_size = size;
		uart_handle->tx_count = size;
		uart_handle->error_code = USART_NO_ERROR;
		uart_handle->global_state = OP_STATE_BUSY_TX;

		usart_interrupt_enable(uart_handle->instance, USART_INT_TBE);

		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

/**
 *  Begin RX transfer (blocking) 
 */
gd_status_enum serial_receive(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size, uint32_t timeout)
{
	uint16_t *tmp;

	if (uart_handle->rx_state == OP_STATE_READY) {
		if ((pData == NULL) || (size == 0U)) {
			return GD_ERROR;
		}

		uart_handle->error_code = USART_NO_ERROR;
		uart_handle->rx_state = OP_STATE_BUSY_RX;
		uart_handle->rx_size = size;
		uart_handle->rx_count = size;

		while (uart_handle->rx_count > 0U) {
			uart_handle->rx_count--;
			if (uart_handle->params.databits == USART_WL_9BIT) {
				if (serial_wait_flag_timeout(uart_handle, USART_FLAG_RBNE, RESET, timeout) != GD_OK) {
					return GD_TIMEOUT;
				}
				tmp = (uint16_t *)pData;
				if (uart_handle->params.parity == USART_PM_NONE) {
					*tmp = (uint16_t)(GET_BITS(USART_DATA(uart_handle->instance), 0U, 8U));
					pData += 2U;
				} else {
					*tmp = (uint16_t)(GET_BITS(USART_DATA(uart_handle->instance), 0U, 7U));
					pData += 1U;
				}
			} else {
				if (serial_wait_flag_timeout(uart_handle, USART_FLAG_RBNE, RESET, timeout) != GD_OK) {
					return GD_TIMEOUT;
				}
				if (uart_handle->params.parity == USART_PM_NONE) {
					*pData++ = (uint8_t)(GET_BITS(USART_DATA(uart_handle->instance), 0U, 7U));
				} else {
					*pData++ = (uint8_t)(GET_BITS(USART_DATA(uart_handle->instance), 0U, 6U));
				}
			}
		}
		uart_handle->rx_state = OP_STATE_READY;

		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

/**
 *  Begin RX transfer (non-blocking) 
 */
gd_status_enum IT_serial_receive(SPL_UartHandle_t *uart_handle, uint8_t *pData, uint16_t size)
{
	if (uart_handle->rx_state == OP_STATE_READY) {
		if ((pData == NULL) || (size == 0U)) {
			return GD_ERROR;
		}

		uart_handle->rx_buffer_ptr = pData;
		uart_handle->rx_size = size;
		uart_handle->rx_count = size;
		uart_handle->error_code = USART_NO_ERROR;
		uart_handle->rx_state = OP_STATE_BUSY_RX;

		usart_interrupt_enable(uart_handle->instance, USART_INT_PERR);
		usart_interrupt_enable(uart_handle->instance, USART_INT_ERR);
		usart_interrupt_enable(uart_handle->instance, USART_INT_RBNE);

		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

gd_status_enum IT_transmit(SPL_UartHandle_t *uart_handle)
{
	uint16_t *tmp;

	if (uart_handle->global_state == OP_STATE_BUSY_TX) {
		if (uart_handle->params.databits == USART_WL_9BIT) {
			tmp = (uint16_t *)uart_handle->tx_buffer_ptr;
			usart_data_transmit(uart_handle->instance, (uint16_t)*tmp);
			if (uart_handle->params.parity == USART_PM_NONE) {
				uart_handle->tx_buffer_ptr += 2U;
			} else {
				uart_handle->tx_buffer_ptr += 1U;
			}
		} else {
			USART_DATA(uart_handle->instance) = (uint8_t)(*uart_handle->tx_buffer_ptr++ & (uint8_t)0xFF);
		}

		if (--uart_handle->tx_count == 0U) {
			usart_interrupt_disable(uart_handle->instance, USART_INT_TBE);
			usart_interrupt_disable(uart_handle->instance, USART_INT_TC);			
		}
		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

gd_status_enum IT_receive(SPL_UartHandle_t *uart_handle)
{
	uint16_t *tmp;
	if (uart_handle->rx_state == OP_STATE_BUSY_RX) {
		if (uart_handle->params.databits == USART_WL_9BIT) {
			tmp = (uint16_t *)uart_handle->rx_buffer_ptr;
			if (uart_handle->params.parity == USART_PM_NONE) {
				*tmp = usart_data_receive(uart_handle->instance);
				uart_handle->rx_buffer_ptr += 2U;
			} else {
				*tmp = (uint16_t)(GET_BITS(USART_DATA(uart_handle->instance), 0U, 7U));
				uart_handle->rx_buffer_ptr += 1U;
			}
		} else {
			if (uart_handle->params.parity == USART_PM_NONE) {
				*uart_handle->rx_buffer_ptr++ = (uint8_t)(GET_BITS(USART_DATA(uart_handle->instance), 0U, 7U));
			} else {
				*uart_handle->rx_buffer_ptr++ = (uint8_t)(GET_BITS(USART_DATA(uart_handle->instance), 0U, 6U));
			}
		}

		if (--uart_handle->rx_count == 0U) {
			usart_interrupt_disable(uart_handle->instance, USART_INT_RBNE);
			usart_interrupt_disable(uart_handle->instance, USART_INT_PERR);
			usart_interrupt_disable(uart_handle->instance, USART_INT_ERR);

			uart_handle->rx_state = OP_STATE_READY;

			UART_RX_TCCallback(uart_handle);
			return GD_OK;
		}
		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

/**
 *  Wait for flag or timeout
 */
gd_status_enum serial_wait_flag_timeout(SPL_UartHandle_t *uart_handle, uint32_t flag, FlagStatus status, uint32_t timeout)
{
	while ((usart_flag_get(uart_handle->instance, flag) ? SET : RESET) == status) {
		for (int i = 0; i < timeout; i++);
		usart_interrupt_disable(uart_handle->instance, USART_INT_RBNE);
		usart_interrupt_disable(uart_handle->instance, USART_INT_PERR);
		usart_interrupt_disable(uart_handle->instance, USART_INT_TBE);
		usart_interrupt_disable(uart_handle->instance, USART_INT_ERR);

		uart_handle->global_state = OP_STATE_READY;
		uart_handle->rx_state = OP_STATE_READY;

		return GD_TIMEOUT;
	}
	return GD_OK;
}

void UART_ErrorCallback(SPL_UartHandle_t *uart_handle)
{
	if (usart_flag_get(uart_handle->instance, USART_FLAG_PERR) != RESET) {
		usart_flag_clear(uart_handle->instance, USART_FLAG_PERR);
	} else if (usart_flag_get(uart_handle->instance, USART_FLAG_FERR) != RESET) {
		usart_flag_clear(uart_handle->instance, USART_FLAG_FERR);
	} else if (usart_flag_get(uart_handle->instance, USART_FLAG_NERR) != RESET) {
		usart_flag_clear(uart_handle->instance, USART_FLAG_NERR);
	} else if (usart_flag_get(uart_handle->instance, USART_FLAG_ORERR) != RESET) {
		usart_flag_clear(uart_handle->instance, USART_FLAG_ORERR);
	}
	serial_t *obj = get_serial_object(uart_handle);
	if (obj && !serial_rx_active(obj)) {
		IT_serial_receive(uart_handle, &(obj->rvalue), 1);
	}
}

void UART_RX_TCCallback(SPL_UartHandle_t *uart_handle)
{
	serial_t *obj = get_serial_object(uart_handle);
	if (obj) {
		obj->rx_callback(obj);
	}
}

void UART_TX_TCCallback(SPL_UartHandle_t *uart_handle)
{
	serial_t *obj = get_serial_object(uart_handle);

	if (obj && obj->tx_callback(obj) != -1) {
		if (IT_serial_transmit(uart_handle, &obj->tx_buff[obj->tx_tail], 1) != GD_OK) {
			return;
		}
	}
}

void serial_rx_transfer_end(SPL_UartHandle_t *uart_handle)
{
	usart_interrupt_disable(uart_handle->instance, USART_INT_RBNE);
	usart_interrupt_disable(uart_handle->instance, USART_INT_PERR);
	usart_interrupt_disable(uart_handle->instance, USART_INT_ERR);

	uart_handle->rx_state = OP_STATE_READY;
}

void IT_serial_transmit_end(SPL_UartHandle_t *uart_handle)
{
	usart_interrupt_disable(uart_handle->instance, USART_INT_TC);
	uart_handle->global_state = OP_STATE_READY;

	UART_TX_TCCallback(uart_handle);
}

/**
 * This function is the uart/usart interrupt handler
 * @param obj_s The serial obj
 */
void UART_IRQHandler(SPL_UartHandle_t *uart_handle)
{
  uint32_t stat_flags = GD32_USART_STAT(uart_handle->instance);
  uint32_t ctl0_bits = USART_CTL0(uart_handle->instance);
  uint32_t ctl2_bits = USART_CTL2(uart_handle->instance);
  uint32_t error_flags = 0U;
  uint32_t dma_request = 0U;

  /* no error occurs */
  error_flags = (stat_flags & (uint32_t)(USART_FLAG_PERR | USART_FLAG_FERR | USART_FLAG_NERR | USART_FLAG_ORERR));
  if (error_flags == RESET) {
    /* check whether USART is in receiver mode */
    if (((stat_flags & USART_FLAG_RBNE) != RESET) && ((ctl0_bits & USART_INT_RBNE) != RESET)) {
    	IT_receive(uart_handle);
      return;
    }
  }

  if ((error_flags != RESET) && (((ctl2_bits & USART_INT_ERR) != RESET) || ((ctl0_bits & (USART_INT_RBNE | USART_INT_PERR)) != RESET))) {
  	if (((stat_flags & USART_FLAG_PERR) != RESET) && ((ctl0_bits & USART_INT_PERR) != RESET)) {
  		uart_handle->error_code |= USART_PARITY_ERROR;
  	}

  	if (((stat_flags & USART_FLAG_NERR) != RESET) && ((ctl2_bits & USART_INT_ERR) != RESET)) {
  		uart_handle->error_code |= USART_NOISE_ERROR;
  	}

  	if (((stat_flags & USART_FLAG_FERR) != RESET) && ((ctl2_bits & USART_INT_ERR) != RESET)) {
  		uart_handle->error_code |= USART_FRAME_ERROR;
  	}

  	if (((stat_flags & USART_FLAG_ORERR) != RESET) && ((ctl2_bits & USART_INT_ERR) != RESET)) {
  		uart_handle->error_code |= USART_OVERRUN_ERROR;
  	}

  	if (uart_handle->error_code != USART_NO_ERROR) {
  		if (((stat_flags & USART_FLAG_RBNE) != RESET) && ((ctl0_bits & USART_INT_RBNE) != RESET)) {
  			IT_receive(uart_handle);
  		}
  		dma_request = ((USART_CTL2(uart_handle->instance) & USART_CTL2_DENR) != 0U);
  		if (((uart_handle->error_code & USART_OVERRUN_ERROR) != RESET) || dma_request) {
  			serial_rx_transfer_end(uart_handle);
  			if ((USART_CTL2(uart_handle->instance) & USART_CTL2_DENR) != 0U) {
  				usart_dma_receive_config(uart_handle->instance, USART_RECEIVE_DMA_DISABLE);
  				UART_ErrorCallback(uart_handle);
  			} else {
  				UART_ErrorCallback(uart_handle);
  			}
  		} else {
  			UART_ErrorCallback(uart_handle);
  		}
  	}
  	return;
  }

  if (((stat_flags & USART_FLAG_TBE) != RESET) && ((ctl0_bits & USART_INT_TBE) != RESET)) {
  	IT_transmit(uart_handle);
  	return;
  }

  if (((stat_flags & USART_FLAG_TC) != RESET) && ((ctl0_bits & USART_INT_TC) != RESET)) {
  	IT_serial_transmit_end(uart_handle);
  	return;
  }
}

void serial_half_duplex_enable_tx(SPL_UartHandle_t *uart_handle)
{
	uart_handle->global_state = OP_STATE_BUSY;
	usart_receive_config(uart_handle->instance, USART_RECEIVE_DISABLE);
	usart_transmit_config(uart_handle->instance, USART_TRANSMIT_ENABLE);
	uart_handle->global_state = OP_STATE_READY;
}

void serial_half_duplex_enable_rx(SPL_UartHandle_t *uart_handle)
{
	uart_handle->global_state = OP_STATE_BUSY;
	usart_transmit_config(uart_handle->instance, USART_TRANSMIT_DISABLE);
	usart_receive_config(uart_handle->instance, USART_RECEIVE_ENABLE);
	uart_handle->global_state = OP_STATE_READY;
}

/**
 * USART0 IRQ handler
 */
#if defined(USART0)
void USART0_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART0_INDEX]);
	UART_IRQHandler(uart_handlers[UART0_INDEX]);
}
#endif

/**
 * USART1 IRQ handler
 */
#if defined(USART1)
void USART1_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART1_INDEX]);
  UART_IRQHandler(uart_handlers[UART1_INDEX]);
}
#endif

/**
 * USART2 IRQ handler
 */
#if defined(USART2)
void USART2_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART2_INDEX]);
	UART_IRQHandler(uart_handlers[UART2_INDEX]);
}
#endif

/**
 * UART3 IRQ handler
 */
#if defined(UART3)
void UART3_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART3_INDEX]);
	UART_IRQHandler(uart_handlers[UART3_INDEX]);
}
#endif

/**
 * USART3 IRQ handler
 */
#if defined(USART3)
void USART3_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART3_INDEX]);
	UART_IRQHandler(uart_handlers[UART3_INDEX]);
}
#endif

/**
 * UART4 IRQ handler
 */
#if defined(UART4)
void UART4_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART4_INDEX]);
	UART_IRQHandler(uart_handlers[UART4_INDEX]);
}
#endif

/**
 * USART4 IRQ handler
 */
#if defined(USART4)
void USART4_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART4_INDEX]);
	UART_IRQHandler(uart_handlers[UART4_INDEX]);
}
#endif

#ifdef __cplusplus
}
#endif
