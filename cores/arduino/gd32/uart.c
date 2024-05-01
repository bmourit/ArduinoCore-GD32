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

#include "uart.h"
#include "Arduino.h"

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

struct serial_s *obj_s_buf[UART_NUM] = { NULL };

static rcu_periph_enum usart_clk[UART_NUM] = {
  RCU_USART0,
  RCU_USART1,
#ifdef USART2
  RCU_USART2,
#endif
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
#ifdef USART2
	USART2_IRQn,
#endif
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

#define GET_SERIAL_S(obj) (obj_s_buf[(obj->index)])

/**
 * Initialize the USART peripheral.
 *
 * @param obj_s The serial object
 */
static void usart_init(struct serial_s *obj_s)
{
	if (obj_s->index >= UART_NUM) {
		return;
	}

	/* USART configuration */
	usart_deinit(obj_s->uart);
	usart_word_length_set(obj_s->uart, obj_s->databits);
	usart_baudrate_set(obj_s->uart, obj_s->baudrate);
	usart_stop_bit_set(obj_s->uart, obj_s->stopbits);
	usart_parity_config(obj_s->uart, obj_s->parity);
	usart_receive_config(obj_s->uart, USART_RECEIVE_ENABLE);
	usart_transmit_config(obj_s->uart, USART_TRANSMIT_ENABLE);
}

/**
 * @brief     transmitter for half duplex mode. NOOP in regular mode
 * @param[in] obj : serial object pointer
 * @retval    none
 */
void serial_enable(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	usart_receive_config(p_obj->uart, USART_RECEIVE_ENABLE);
  usart_transmit_config(p_obj->uart, USART_TRANSMIT_ENABLE);
  usart_enable(p_obj->uart);
}

/**
 * Initialize the serial peripheral. It sets the default parameters for serial
 * peripheral, and configures the specified pins.
 *
 * @param obj The serial object
 * @param tx  The TX pin name
 * @param rx  The RX pin name
 */
void serial_init(serial_t *obj, PinName tx, PinName rx)
{
  //struct serial_s *p_obj = obj; //GET_SERIAL_S(obj);
  if (obj == NULL) {
    return;
  }

	UARTName uart_tx = (UARTName)pinmap_peripheral(tx, PinMap_UART_TX);
	UARTName uart_rx = (UARTName)pinmap_peripheral(rx, PinMap_UART_RX);

	obj->uart = (UARTName)pinmap_merge(uart_tx, uart_rx);

	/* set uart index */
	switch (obj->uart) {
#if defined(USART0)
		case USART0:
			obj->index = UART0_INDEX;
			break;
#endif
#if defined(USART1)
		case USART1:
			obj->index = UART1_INDEX;
			break;
#endif
#if defined(USART2)
		case USART2:
			obj->index = UART2_INDEX;
			break;
#endif
#if defined(UART3)
		case UART3:
			obj->index = UART3_INDEX;
			break;
#endif
#if defined(UART4)
		case UART4:
			obj->index = UART4_INDEX;
			break;
#endif
	}

	/* reset and enable UART peripheral clock */
  usart_deinit(obj->uart);
	rcu_periph_clock_enable(usart_clk[obj->index]);

	/* configure the pins */
	pinmap_pinout(tx, PinMap_UART_TX);
  pinmap_pinout(rx, PinMap_UART_RX);

	obj->baudrate = 9600U;
	obj->databits = USART_WL_8BIT;
	obj->stopbits = USART_STB_1BIT;
	obj->parity = USART_PM_NONE;
	obj->pin_tx = tx;
	obj->pin_rx = rx;
	obj->tx_state = OP_STATE_BUSY;
	obj->rx_state = OP_STATE_BUSY;

	obj_s_buf[obj->index] = obj;
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	usart_init(p_obj);

	p_obj->tx_state = OP_STATE_READY;
	p_obj->rx_state = OP_STATE_READY;
}

/**
 * Release the serial peripheral, not currently invoked. It requires further
 *  resource management.
 *
 * @param obj The serial object
 */
void serial_free(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

  /* reset usart/uart and disable clock */
	usart_deinit(p_obj->uart);
	rcu_periph_clock_disable(usart_clk[p_obj->index]);

  p_obj->tx_state = OP_STATE_BUSY;
  usart_disable(p_obj->uart);

  p_obj->error_code = USART_NO_ERROR;
  p_obj->tx_state = OP_STATE_RESET;
  p_obj->rx_state = OP_STATE_RESET;

	/* reset the gpio state */
#if defined(GD32F30x) || defined(GD32F10x)|| defined(GD32E50X)
	pin_function(p_obj->pin_tx, PIN_PUPD_NONE);
	pin_function(p_obj->pin_rx, PIN_PUPD_NONE);
#else
	pin_function(p_obj->pin_tx, PIN_MODE_INPUT);
	pin_function(p_obj->pin_rx, PIN_MODE_INPUT);
#endif
}

/**
 * Configure the baud rate
 *
 * @param obj      The serial object
 * @param baudrate The baud rate to be configured
 */
void serial_baud(serial_t *obj, int baudrate)
{
	uint16_t uen_flag = 0U;
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* store the uen flag */
	uen_flag = USART_CTL0(p_obj->uart) & USART_CTL0_UEN;

	/* disable the uart first */
	usart_disable(p_obj->uart);
	usart_baudrate_set(p_obj->uart, baudrate);

	p_obj->baudrate = baudrate;

	/* restore the uen flag */
	if (RESET != uen_flag) {
		usart_enable(p_obj->uart);
	}
}

/**
 * Configure the format. Set the number of bits, parity and the number of stop bits
 *
 * @param obj       The serial object
 * @param data_bits The number of data bits
 * @param parity    The parity
 * @param stop_bits The number of stop bits
 */
void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
{
	uint16_t uen_flag = 0U;
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* store the uen flag */
	uen_flag = USART_CTL0(p_obj->uart) & USART_CTL0_UEN;

	/* disable the UART clock first */
	usart_disable(p_obj->uart);

	/* configurate the UART parity */
	switch (parity) {
		case ParityOdd:
			p_obj->parity = USART_PM_ODD;
			break;
		case ParityEven:
			p_obj->parity = USART_PM_EVEN;
			break;
		case ParityNone:
		case ParityForced0:
		case ParityForced1:
		default:
			p_obj->parity = USART_PM_NONE;
			break;
	}
	usart_parity_config(p_obj->uart, p_obj->parity);

	if (p_obj->parity == USART_PM_NONE) {
		switch (data_bits) {
			case 9:
				p_obj->databits = USART_WL_9BIT;
				break;
			case 8:
				p_obj->databits = USART_WL_8BIT;
				break;
			case 7:
				return;
		}
		usart_word_length_set(p_obj->uart, p_obj->databits);
	} else {
		switch (data_bits) {
			case 9:
				return;
			case 8: 
				p_obj->databits = USART_WL_9BIT;
				break;
			case 7:
				p_obj->databits = USART_WL_8BIT;
				break;
		}
		usart_word_length_set(p_obj->uart, p_obj->databits);
	}

	if (stop_bits == 2) {
		p_obj->stopbits = USART_STB_2BIT;
	} else {
		p_obj->stopbits = USART_STB_1BIT;
	}
	usart_stop_bit_set(p_obj->uart, p_obj->stopbits);

	/* restore the uen flag */
	if (RESET != uen_flag) {
		usart_enable(p_obj->uart);
	}
}

/**
 * @brief   Reads a rx byte from uart (blocking call)
 * @param   obj serial object pointer
 * @retval  returns the last character received
 */
int serial_getc(serial_t *obj)
{
  struct serial_s *p_obj = GET_SERIAL_S(obj);

	return (int)(usart_data_receive(p_obj->uart) & BITS(0, 7 + (p_obj->databits >> 12)));
}

/**
 * Send a character. This is a blocking call, waiting for a peripheral to be available
 *  for writing
 *
 * @param obj The serial object
 * @param c   The character to be sent
 */
void serial_putc(serial_t *obj, int c)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	while (!serial_writable(obj));
	usart_data_transmit(p_obj->uart, (int)((c) & BITS(0, 7 + (p_obj->databits >> 12))));
}

/**
 * Check if the serial peripheral is readable
 *
 * @param obj The serial object
 * @return Non-zero value if a character can be read, 0 if nothing to read
 */
int serial_readable(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	return (usart_flag_get(p_obj->uart, USART_FLAG_RBNE) != RESET) ? 1 : 0;
}

/**
 * Check if the serial peripheral is writable
 *
 * @param obj The serial object
 * @return Non-zero value if a character can be written, 0 otherwise.
 */
int serial_writable(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	return (usart_flag_get(p_obj->uart, USART_FLAG_TBE) != RESET) ? 1 : 0;
}

/**
 * Clear the serial peripheral
 *
 * @param obj The serial object
 */
void serial_clear(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	p_obj->tx_count = 0U;
	p_obj->rx_count = 0U;
}

/**
 * Tries to determine if the serial is already being used for tx
 * @param obj serial object
 * @retval returns 1 if rx transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	return ((p_obj->tx_state == OP_STATE_BUSY_TX) ? 1 : 0);
}

/**
 * Tries to determine if the serial is already being used for rx
 * @param obj serial object
 * @retval returns 1 if rx transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	return ((p_obj->rx_state == OP_STATE_BUSY_RX) ? 1 : 0);
}

/**
 * Attach UART transmit callback
 *
 * @param obj The serial object
 * @param callback The transmit callback
 */
void uart_attach_tx_callback(serial_t *obj, void(*callback)(serial_t *))
{
	if (obj == NULL) {
		return;
	}
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	if (serial_tx_active(p_obj)) {
		return;
	}
	p_obj->tx_callback = callback;
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
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* Exit if a reception is already on-going */
	if (serial_rx_active(p_obj)) {
		return;
	}
	p_obj->rx_callback = callback;
}

/**
 * Handle the serial rx interrupt
 *
 * @param obj_s The serial object
 * @return Returns the status
 */
static gd_status_enum usart_rx_interrupt(struct serial_s *obj_s)
{
	uint16_t *temp;

	if (obj_s->rx_state == OP_STATE_BUSY_RX) {
		if (obj_s->databits == USART_WL_9BIT) {
			temp = (uint16_t *)obj_s->rx_buffer_ptr;
			if (obj_s->parity == USART_PM_NONE) {
				/* 9-bit data, none parity bit */
				*temp = (uint16_t)(GD32_USART_RX_DATA(obj_s->uart) & (uint16_t)0x01FF);
				obj_s->rx_buffer_ptr += 2U;
			} else {
				/* 9-bit data, with parity bit */
				*temp = (uint16_t)(GD32_USART_RX_DATA(obj_s->uart) & (uint16_t)0x00FF);
				obj_s->rx_buffer_ptr += 1U;
			}
		} else {
			if (obj_s->parity == USART_PM_NONE) {
				/* 8-bit data, none parity bit */
				*obj_s->rx_buffer_ptr++ = (uint8_t)(GD32_USART_RX_DATA(obj_s->uart) & (uint8_t)0x00FF);
			} else {
				/* 8-bit data, with parity bit */
				*obj_s->rx_buffer_ptr++ = (uint8_t)(GD32_USART_RX_DATA(obj_s->uart) & (uint8_t)0x007F);
			}
		}

		if (--obj_s->rx_count == 0U) {
			usart_interrupt_disable(obj_s->uart, USART_INT_RBNE);
			usart_interrupt_disable(obj_s->uart, USART_INT_PERR);
			usart_interrupt_disable(obj_s->uart, USART_INT_ERR);

			obj_s->rx_state = OP_STATE_READY;
			obj_s->rx_callback(obj_s);
		}
		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

/**
 * Handle the serial tx interrupt
 *
 * @param obj_s The serial object
 * @return Returns the status
 */
static gd_status_enum usart_tx_interrupt(struct serial_s *obj_s)
{
	uint16_t *temp;

	if (obj_s->tx_state == OP_STATE_BUSY_TX) {
		if (obj_s->databits == USART_WL_9BIT) {
			temp = (uint16_t *) obj_s->tx_buffer_ptr;
			GD32_USART_TX_DATA(obj_s->uart) = (uint16_t)(*temp & (uint16_t)0x01FF);
			if (obj_s->parity == USART_PM_NONE) {
				obj_s->tx_buffer_ptr += 2U;
			} else {
				obj_s->tx_buffer_ptr += 1U;
			}
		} else {
			GD32_USART_TX_DATA(obj_s->uart) = (uint8_t)(*obj_s->tx_buffer_ptr++ & (uint8_t)0x00FF);
		}

		if (--obj_s->tx_count == 0U) {
			/* disable USART_INT_TBE interrupt */
			usart_interrupt_disable(obj_s->uart, USART_INT_TBE);
			/* enable USART_INT_TC interrupt */
			usart_interrupt_enable(obj_s->uart, USART_INT_TC);
		}
		return GD_OK;
	} else {
		return GD_BUSY;
	}
}

/**
 * Handle the serial tx complete interrupt
 *
 * @param obj_s The serial object
 */
static void usart_tx_complete_interrupt(struct serial_s *obj_s)
{
	usart_interrupt_disable(obj_s->uart, USART_INT_TC);

	obj_s->tx_state = OP_STATE_READY;
	obj_s->tx_callback(obj_s);
}

/**
 * Preprocess the USART tx interrupt
 *
 * @param obj_s The serial object
 * @param pData Pointer to tx buffer
 * @param Size  Size of tx buffer
 * @return Returns the status
 */
static gd_status_enum usart_tx_interrupt_preprocess(struct serial_s *obj_s, uint8_t *pData, uint16_t Size)
{
    if (obj_s->tx_state == OP_STATE_READY) {
        if ((pData == NULL) || (Size == 0U)) {
            return GD_ERROR;
        }

        obj_s->tx_buffer_ptr = pData;
        obj_s->tx_count = Size;
        obj_s->tx_state = OP_STATE_BUSY_TX;

        usart_interrupt_enable(obj_s->uart, USART_INT_TBE);
        return GD_OK;
    } else {
        return GD_BUSY;
    }
}

/**
 * Preprocess the USART rx interrupt
 *
 * @param obj_s The serial object
 * @param pData Pointer to rx buffer
 * @param Size  Size of rx buffer
 * @return Returns the status
 */
static gd_status_enum usart_rx_interrupt_preprocess(struct serial_s *obj_s, uint8_t *pData,
                                                    uint16_t Size)
{
    if (obj_s->rx_state == OP_STATE_READY) {
        if ((pData == NULL) || (Size == 0U)) {
            return GD_ERROR;
        }

        obj_s->rx_buffer_ptr = pData;
        obj_s->rx_size = Size;
        obj_s->rx_count = Size;
        obj_s->rx_state = OP_STATE_BUSY_RX;

        usart_interrupt_enable(obj_s->uart, USART_INT_PERR);
        usart_interrupt_enable(obj_s->uart, USART_INT_ERR);
        usart_interrupt_enable(obj_s->uart, USART_INT_RBNE);
        return GD_OK;
    } else {
        return GD_BUSY;
    }
}

/**
 * Begin asynchronous TX transfer
 * @param obj       The serial object
 * @param tx        The transmit buffer
 * @param tx_length The number of bytes to transmit
 * @return Returns number of data transfered, otherwise returns 0
 */
int serial_transmit(serial_t *obj, const uint8_t *tx, size_t tx_length)
{
  struct serial_s *p_obj = GET_SERIAL_S(obj);
  IRQn_Type irq = usart_irq_n[p_obj->index];

  if (tx_length == 0) {
      return 0;
  }

  if (serial_tx_active(obj)) {
      /* some transmit is in progress */
      return 0;
  }

  obj->tx_buffer_ptr = (uint8_t *)tx;
  obj->tx_count = tx_length;

  /* enable interrupt */
  /* clear pending IRQ */
  NVIC_ClearPendingIRQ(irq);
  /* disable the IRQ first */
  nvic_irq_disable(irq);
  /* enable IRQ */
  nvic_irq_enable(irq, 1, 0);

  if (usart_tx_interrupt_preprocess(p_obj, (uint8_t *)tx, tx_length) != GD_OK) {
      return 0;
  }

  return tx_length;
}

/**
 *  Begin RX transfer (blocking) 
 */
void serial_receive(serial_t *obj, const uint8_t *rx, size_t rx_length)
{
  struct serial_s *p_obj = GET_SERIAL_S(obj);
  IRQn_Type irq = usart_irq_n[p_obj->index];

  if (rx_length == 0) {
      return;
  }

  if (serial_rx_active(obj)) {
		/* some reception is in progress */
		return;
  }

	obj->rx_buffer_ptr = (uint8_t *)rx;
	obj->rx_count = rx_length;

	/* enable interrupt */
	/* clear pending irq */
	NVIC_ClearPendingIRQ(irq);
	/* disable the irq first */
	nvic_irq_disable(irq);
	/* enable irq */
	nvic_irq_enable(irq, 0, 0);

	usart_rx_interrupt_preprocess(p_obj, (uint8_t *)rx, rx_length);
}

/**
 * This function is the uart/usart interrupt handler
 * @param obj_s The serial obj
 */
static void UART_IRQHandler(struct serial_s *obj_s)
{
  uint32_t err_flags = 0U;

  /* no error occurs */
  err_flags = (GD32_USART_STAT(obj_s->uart) & (uint32_t)(USART_FLAG_PERR | USART_FLAG_FERR |
                                               USART_FLAG_ORERR | USART_FLAG_NERR));
  if (err_flags == RESET) {
    /* check whether USART is in receiver mode or not */
    if (usart_interrupt_flag_get(obj_s->uart, USART_INT_FLAG_RBNE) != RESET) {
      usart_interrupt_flag_clear(obj_s->uart, USART_INT_FLAG_RBNE);
      usart_rx_interrupt(obj_s);
      return;
    }
  }
  if (usart_interrupt_flag_get(obj_s->uart, USART_INT_FLAG_TBE) != RESET) {
    usart_tx_interrupt(obj_s);
    return;
  }
  if (usart_interrupt_flag_get(obj_s->uart, USART_INT_FLAG_TC) != RESET) {
    usart_interrupt_flag_clear(obj_s->uart, USART_INT_FLAG_TC);
    usart_tx_complete_interrupt(obj_s);
    return;
  }
  if (usart_interrupt_flag_get(obj_s->uart, USART_INT_FLAG_ERR_ORERR) != RESET) {
    /* clear ORERR error flag by reading USART DATA register */
    GD32_USART_RX_DATA(obj_s->uart);
    usart_interrupt_flag_clear(obj_s->uart, USART_INT_FLAG_ERR_ORERR);
  }
  if (usart_interrupt_flag_get(obj_s->uart, USART_INT_FLAG_ERR_NERR) != RESET) {
    /* clear NERR error flag by reading USART DATA register */
    GD32_USART_RX_DATA(obj_s->uart);
    usart_interrupt_flag_clear(obj_s->uart, USART_INT_FLAG_ERR_NERR);
  }
  if (usart_interrupt_flag_get(obj_s->uart, USART_INT_FLAG_ERR_FERR) != RESET) {
    /* clear FERR error flag by reading USART DATA register */
    GD32_USART_RX_DATA(obj_s->uart);
    /* also clear it by clearing the interrupt */
    usart_interrupt_flag_clear(obj_s->uart, USART_INT_FLAG_ERR_FERR);
  }

  if (usart_interrupt_flag_get(obj_s->uart, USART_INT_FLAG_PERR) != RESET) {
    /* clear PERR error flag by reading USART DATA register */
    GD32_USART_RX_DATA(obj_s->uart);
    /* also clear it by clearing the interrupt */
    usart_interrupt_flag_clear(obj_s->uart, USART_INT_FLAG_PERR);
  }
}

/**
 * USART0 IRQ handler
 */
#if defined(USART0)
void USART0_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART0_INDEX]);
	UART_IRQHandler(obj_s_buf[UART0_INDEX]);
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
  UART_IRQHandler(obj_s_buf[UART1_INDEX]);
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
	UART_IRQHandler(obj_s_buf[UART2_INDEX]);
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
	UART_IRQHandler(obj_s_buf[UART3_INDEX]);
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
	UART_IRQHandler(obj_s_buf[UART3_INDEX]);
}
#endif

/**
 * UART4 RQ handler
 */
#if defined(UART4)
void UART4_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART4_INDEX]);
	UART_IRQHandler(obj_s_buf[UART4_INDEX]);
}
#endif

/**
 * USART4 RQ handler
 */
#if defined(USART4)
void USART4_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART4_INDEX]);
	UART_IRQHandler(obj_s_buf[UART4_INDEX]);
}
#endif

#ifdef __cplusplus
}
#endif
