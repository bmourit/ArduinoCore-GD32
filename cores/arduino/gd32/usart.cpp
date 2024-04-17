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

/* set up a default usart/uart for debug */
#if !defined(SERIAL_DEBUG)
#if defined (PIN_SERIAL_TX)
#define SERIAL_DEBUG      pinmap_peripheral(digitalPinToPinName(PIN_SERIAL_TX), PinMap_UART_TX)
#define DEBUG_TX_PINNAME  digitalPinToPinName(PIN_SERIAL_TX)
#else
#define SERIAL_DEBUG      NC
#define DEBUG_TX_PINNAME  NC
#endif
#endif
#if !defined(SERIAL_DEBUG_BAUD)
#define SERIAL_DEBUG_BAUD 9600
#endif

struct serial_s *uart_handler_buf[NUM_UART] = { NULL };

static serial_t is_debug = {
  .uart = NC,
  .index = NUM_UART
};

static rcu_periph_enum usart_clk[NUM_UART] = {
  RCU_USART0,
  RCU_USART1,
#ifdef USART2
  RCU_USART2,
#endif
#ifdef USART3
  RCU_UART3,
#endif
#ifdef USART4
  RCU_UART4
#endif
};

static rcu_periph_reset_enum usart_rst[NUM_UART] = {
  RCU_USART0RST,
  RCU_USART1RST,
#ifdef USART2
  RCU_USART2RST,
#endif
#ifdef USART3
  RCU_UART3RST,
#endif
#ifdef USART4
  RCU_UART4RST
#endif
};

static IRQn_Type usart_irq_n[NUM_UART] = {
	USART0_IRQn,
	USART1_IRQn,
#ifdef USART2
	USART2_IRQn,
#endif
#ifdef USART3
	UART3_IRQn,
#endif
#ifdef USART4
	UART4_IRQn
#endif
};

#define GET_SERIAL_S(obj) 	uart_handler_buf(obj->index)

/**
 * Initialize the USART peripheral.
 *
 * @param obj_s The serial object
 */
static void uart_init(struct serial_s *hwSerial)
{
	if (hwSerial->index >= NUM_UART) {
		return;
	}

	/* USART configuration */
	uart_deinit(hwSerial->uart);
  /* check for half duplex mode */
  if (hwSerial->halfduplex) {
    usart_halfduplex_enable(hwSerial->uart);
  }

  usart_word_length_set(hwSerial->uart, USART_WL_8BIT);
  usart_baudrate_set(hwSerial->uart, hwSerial->baudrate);
  usart_stop_bit_set(hwSerial->uart, USART_STB_1BIT);
  usart_parity_config(hwSerial->uart, USART_PM_NONE);
  usart_receive_config(hwSerial->uart, USART_RECEIVE_ENABLE);
  usart_transmit_config(hwSerial->uart, USART_TRANSMIT_ENABLE);

  usart_enable(hwSerial->uart);	
}


/**
 * @brief     transmitter for half duplex mode. NOOP in regular mode
 * @param[in] obj : serial object pointer
 * @retval    none
 */
void serial_enable_tx(serial_t *obj)
{
	struct serial_s *hwSerial = GET_SERIAL_S(obj);
	usart_transmit_config(hwSerial->uart, USART_TRANSMIT_ENABLE);
}

/**
 * @brief     receiver for half duplex mode. NOOP in regular mode
 * @param[in] obj : serial object pointer
 * @retval    none
 */
void serial_enable_rx(serial_t *obj)
{
	struct serial_s *hwSerial = GET_SERIAL_S(obj);
	usart_receive_config(hwSerial->uart, USART_RECEIVE_ENABLE);
}


/**
 * Initialize the serial peripheral. It sets the default parameters for serial
 * peripheral, and configures its specifieds pins.
 *
 * @param obj      The serial object
 * @param tx       The TX pin name
 * @param rx       The RX pin name
 * @param baudrate The baudrate
 * @param databits The number of data bits
 * @param parity   The parity
 * @param stopbits The number of stop bits
 */
void serial_init(serial_t *p_obj, PinName tx, PinName rx, uint32_t baudrate, uint32_t databits, SerialParity parity, uint32_t stopbits)
{
	struct serial_s *hwSerial = GET_SERIAL_S(p_obj);

	/* enable GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOC);

	/* enable USART clock */
	rcu_periph_clock_enable(usart_clock[hwSerial->index]);

	/* connect port to USARTx_Tx */
	gpio_init(tx, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

	/* connect port to USARTx_Rx */
	gpio_init(rx, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

	/* USART configuration */
	uart_init(hwSerial);

	hwSerial->baudrate = baudrate;

	/* set the default baud rate */
	serial_baud(p_obj, baudrate);

	/* set the default format */
	serial_format(p_obj, databits, parity, stopbits);
}


/**
 * 
 * 
 */
void uart_deinit(serial_t *obj)
{
	struct serial_s *hwSerial = GET_SERIAL_S(obj);

	/* disable the USART first */
	usart_disable(hwSerial->uart);

	/* reset the UART peripheral */
	rcu_periph_reset_enable(usart_rst[hwSerial->index]);
	rcu_periph_reset_disable(usart_rst[hwSerial->index]);

	/* disable USART clock */
	rcu_periph_clock_disable(usart_clock[hwSerial->index]);

	/* reset the GPIO pins */
	gpio_init(obj->pin_tx, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_RESET);
	gpio_init(obj->pin_rx, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_RESET);
}

/**
  * @brief  Initializes a debug uart one if not alread initialized
  *         Default config: 8N1
  * @retval none
  */
void serial_debug_init(void)
{
	if (serial_debug_inited == false) {
		serial_t serial_debug = SERIAL_DEBUG;
		serial_debug_inited = true;

		/* enable GPIO clock */
		rcu_periph_clock_enable(RCU_GPIOA);

		/* enable USART clock */
		rcu_periph_clock_enable(usart_clock[serial_debug.index]);

		/* connect port to USARTx_Tx */
		gpio_init(serial_debug.pin_tx, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

		/* connect port to USARTx_Rx */
		gpio_init(serial_debug.pin_rx, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

		/* USART configuration */
		usart_deinit(serial_debug.uart);

		usart_baudrate_set(serial_debug.uart, 115200U);
		usart_word_length_set(serial_debug.uart, USART_WL_8BIT);
		usart_stop_bit_set(serial_debug.uart, USART_STB_1BIT);
		usart_parity_config(serial_debug.uart, USART_PM_NONE);

		/* enable USART */
		usart_enable(serial_debug.uart);
	}
}


/**
  * @brief  write the data for debug only (syscalls)
  * @param  data
  * @param  size of data
  * @retval returns the number written
  */
size_t serial_debug_write(uint8_t *data, uint32_t size)
{
	serial_t serial_debug = SERIAL_DEBUG;
	uint32_t count = 0U;

	if ((NULL == data) || (0U == size)) {
		return 0U;
	}

	/* wait until the tx data register is empty */
	while (RESET == usart_flag_get(serial_debug.uart, USART_FLAG_TBE)) {
	}

	for (count = 0U; count < size; count++) {
		/* wait until the tx data register is empty */
		while (RESET == usart_flag_get(serial_debug.uart, USART_FLAG_TBE)) {
		}
		/* send data */
		usart_data_transmit(serial_debug.uart, data[count]);
	}

	return count;
}


/**
 * Release the serial peripheral
 *
 * @param obj The serial object
 */
void serial_free(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* disable USART */
	usart_disable(p_obj->uart);

	/* reset USART */
	rcu_periph_reset_enable(usart_rst[p_obj->index]);
	rcu_periph_reset_disable(usart_rst[p_obj->index]);

	/* GPIO pin: TO BE IMPLEMENTED if necessary ! */
}


/**
 * Configure the baud rate
 *
 * @param obj      The serial object
 * @param baudrate The baud rate to be configured
 */
void serial_baud(serial_t *obj, int baudrate)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	uint16_t uen_flag = 0U;

	/* store the UEN flag */
	uen_flag = USART_CTL0(p_obj->uart) & USART_CTL0_UEN;

	/* disable the USART first */
	usart_disable(p_obj->uart);

	usart_baudrate_set(p_obj->uart, baudrate);

	p_obj->baudrate = baudrate;

	/* restore the UEN flag */
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
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	uint32_t reg_val = 0U;

	/* disable the USART first */
	usart_disable(p_obj->uart);

	reg_val = USART_CTL0(p_obj->uart);
	reg_val &= ~(USART_CTL0_UEN | USART_CTL0_WL);

	/* set the data bits */
	if (data_bits == 9) {
		reg_val |= (USART_CTL0_UEN | USART_CTL0_WL);
	}

	/* set the parity */
	switch (parity) {
		case ParityNone:
			reg_val &= ~(USART_CTL0_PEN | USART_CTL0_PM);
			break;
		case ParityOdd:
			reg_val |= USART_CTL0_PEN;
			reg_val &= ~USART_CTL0_PM;
			break;
		case ParityEven:
			reg_val |= (USART_CTL0_PEN | USART_CTL0_PM);
			break;
	}

	/* set the number of stop bits */
	if (stop_bits == 2) {
		reg_val |= USART_CTL0_STB;
	} else {
		reg_val &= ~USART_CTL0_STB;
	}

	USART_CTL0(p_obj->uart) = reg_val;

	p_obj->databits = data_bits;
	p_obj->parity = parity;
	p_obj->stopbits = stop_bits;

	usart_enable(p_obj->uart);
}


/**
 * @brief   Reads a rx byte from uart
 * @param   obj serial object pointer
 * @retval  returns the last character received
 */
int serial_getc(serial_t *obj, unsigned char *c)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* check the status of RBNE bit */
	if (usart_flag_get(p_obj->uart, USART_FLAG_RBNE) != RESET) {
		/* read one byte from the data register */
		*c = usart_data_receive(p_obj->uart);
		return 1;
	}

	return 0;
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

	/* wait the transmit data register empty */
	while (usart_flag_get(p_obj->uart, USART_FLAG_TBE) == RESET) {}

	/* write one byte to the transmit data register */
	usart_data_transmit(p_obj->uart, (uint8_t)c);
}


size_t uart_write(serial_t *obj, const void *data, size_t size)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	size_t n = 0;

	while (n < size) {
		serial_putc(obj, *((uint8_t *)data + n));
		n++;
	}

	return n;
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

	if (usart_flag_get(p_obj->uart, USART_FLAG_RBNE) != RESET) {
		return 1;
	}

	return 0;
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

	if (usart_flag_get(p_obj->uart, USART_FLAG_TBE) != RESET) {
		return 1;
	}

	return 0;
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

	/* clear RXNE and TC flag */
	usart_flag_clear(p_obj->uart, USART_FLAG_RBNE);
	usart_flag_clear(p_obj->uart, USART_FLAG_TC);
}

/**
 * Tries to determine if the serial is already being used for tx
 * @param obj serial object
 * @retval returns 1 if tx transaction is ongoing, 0 otherwise
 */
uint8_t serial_tx_active(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	if (p_obj->tx_state == OP_STATE_BUSY_TX) {
		return 1U;
	}

	return 0U;
}

/**
 * Tries to determine if the serial is already being used for rx
 * @param obj serial object
 * @retval returns 1 if rx transaction is ongoing, 0 otherwise
 */
uint8_t serial_rx_active(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* check the status of RBNE bit */
	if (usart_flag_get(p_obj->uart, USART_FLAG_RBNE) != RESET) {
		return 1U;
	}

	return 0U;
}

/**
 * Attach UART transmit callback
 *
 * @param obj The serial object
 * @param callback The transmit callback
 */
void uart_attach_tx_callback(serial_t *obj, void (*callback)(serial_t *))
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	p_obj->tx_callback = callback;
}

/**
 * Attach UART receive callback
 *
 * @param obj The serial object
 * @param callback The transmit callback
 */
void uart_attach_rx_callback(serial_t *obj, void (*callback)(serial_t *))
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	p_obj->rx_callback = callback;
}

/**
 * Handle the serial rx interrupt
 *
 * @param obj_s The serial object
 * @return Returns the status
 */
static gd_status_enum uart_rx_interrupt(struct serial_s *obj_s)
{
	uint8_t ch;
	gd_status_enum retval = GD_OK;

	ch = usart_data_receive(obj_s->uart);

	obj_s->rx_buff[obj_s->rx_head] = ch;
	obj_s->rx_head++;
	obj_s->rx_count++;
	if (obj_s->rx_head == SERIAL_RX_BUFF_SIZE) {
		obj_s->rx_head = 0U;
	}

	if (obj_s->rx_count == SERIAL_RX_BUFF_SIZE) {
		retval = GD_ERR_FAIL;
	}

	if (NULL != obj_s->rx_callback) {
		obj_s->rx_callback(obj_s);
	}

	return retval;
}


/**
 * Handle the serial tx interrupt
 *
 * @param obj_s The serial object
 * @return Returns the status
 */
static gd_status_enum uart_tx_interrupt(struct serial_s *obj_s)
{
	gd_status_enum retval = GD_OK;

	if (obj_s->tx_count > 0) {
		usart_data_transmit(obj_s->uart, obj_s->tx_buff[obj_s->tx_tail]);
		obj_s->tx_tail = (obj_s->tx_tail + 1) % SERIAL_TX_BUFF_SIZE;
		obj_s->tx_count--;
	} else {
		usart_interrupt_disable(obj_s->uart, USART_INT_TBE);
		retval = GD_ERR_FAIL;
	}

	return retval;
}


/**
 * Handle the serial tx complete interrupt
 *
 * @param obj_s The serial object
 */
static void usart_tx_complete_interrupt(struct serial_s *obj_s)
{
	/* disable the TBE interrupt */
	usart_interrupt_disable(obj_s->uart, USART_INT_TC);

	/* call the callback if it is set */
	if (NULL != obj_s->tx_done_callback) {
		obj_s->tx_done_callback(obj_s);
	}
}


static gd_status_enum uart_wait_flag_timeout(serial_t *obj, usart_flag_enum flag)
{
	gd_status_enum retval = GD_OK;
	uint32_t tickstart = 0U;

	/* get tick */
	tickstart = rt_tick_get();

	/* wait until the flag is set */
	while (RESET == usart_flag_get(obj->uart, flag)) {
		/* check for the timeout */
		if (rt_tick_get() - tickstart > RT_TICK_PER_SECOND) {
			retval = GD_ERR_TIMEOUT;
			break;
		}
	}

	return retval;
}


/**
 *  Begin TX transfer (blocking) 
 */
gd_status_enum serial_transmit(serial_t *obj, const uint8_t *txData, size_t txsize)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	gd_status_enum retval = GD_OK;
	uint32_t tickstart = 0U;

	/* start sending data */
	while ((txsize > 0U) && (GD_OK == retval)) {
		/* wait until the TBE is set */
		retval = uart_wait_flag_timeout(obj, USART_FLAG_TBE);
		if (GD_OK == retval) {
			usart_data_transmit(p_obj->uart, *txData);
			txData++;
			txsize--;
		}
	}

	/* wait until the TC is set */
	tickstart = rt_tick_get();
	while (RESET == usart_flag_get(p_obj->uart, USART_FLAG_TC)) {
		/* check for the timeout */
		if (rt_tick_get() - tickstart > RT_TICK_PER_SECOND) {
			retval = GD_ERR_TIMEOUT;
			break;
		}
	}

	return retval;
}


/**
 *  Begin RX transfer (blocking) 
 */
gd_status_enum serial_receive(serial_t *obj, const uint8_t *rxData, size_t rxsize)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	gd_status_enum retval = GD_OK;
	uint32_t tickstart = 0U;

	/* start receiving data */
	while ((rxsize > 0U) && (GD_OK == retval)) {
		/* wait until the RBNE is set */
		retval = uart_wait_flag_timeout(obj, USART_FLAG_RBNE);
		if (GD_OK == retval) {
			*rxData = usart_data_receive(p_obj->uart);
			rxData++;
			rxsize--;
		}
	}

	/* wait until the REA is set */
	tickstart = rt_tick_get();
	while (RESET == usart_flag_get(p_obj->uart, USART_FLAG_REA)) {
		/* check for the timeout */
		if (rt_tick_get() - tickstart > RT_TICK_PER_SECOND) {
			retval = GD_ERR_TIMEOUT;
			break;
		}
	}

	return retval;
}


/* The following transmit and receive functions are non-blocking */

/**
 * Begin asynchronous TX transfer.
 *
 * @param obj       The serial object
 * @param txData        The transmit buffer
 * @param txSize The number of bytes to transmit
 * @return Returns number of data transfered, otherwise returns 0
 */
gd_status_enum serial_transmit_nb(serial_t *obj, const uint8_t *txData, size_t txSize)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);
	gd_status_enum retval = GD_OK;
	uint32_t sent_count = 0U;

	/* start sending data */
	while ((txSize > 0U) && (GD_OK == retval)) {
		/* wait until the TBE is set */
		retval = uart_wait_flag_timeout(obj, USART_FLAG_TBE);
		if (GD_OK == retval) {
			usart_data_transmit(p_obj->uart, *txData);
			txData++;
			txSize--;
			sent_count++;
		}
	}

	return (retval == GD_OK) ? sent_count : 0;
}


/**
 * Begin asynchronous RX transfer (enable interrupt for data collecting).
 *
 * @param obj        The serial object
 * @param rxData         The receive buffer
 * @param rxSize  The number of bytes to receive
 */
gd_status_enum serial_receive_nb(serial_t *obj, uint8_t *rxData, size_t rxSize)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* enable the USART RBNE interrupt */
	usart_interrupt_enable(p_obj->uart, USART_INT_RBNE);

	p_obj->rx_param.data_length = rxSize;
	p_obj->rx_param.prxtx_buff = rxData;

	return GD_OK;
}

/* The following transfer complete and error callbacks are all non-blocking */

/**
  * @brief  rx transfer complete callback
  * @param  serial obj serial object
  * @retval none
  */
void uart_rx_transfercomplete(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* copy received data to buffer */
	uint32_t i = 0U;
	for (i = 0U; i < p_obj->rx_param.data_length; i++) {
		p_obj->rx_param.prxtx_buff[i] = usart_data_receive(p_obj->uart);
	}

	p_obj->rx_state = OP_STATE_READY;
}


/**
  * @brief  tx transfer complete callback
  * @param  serial obj serial object
  * @retval none
  */
void uart_tx_transfercomplete(serial_t *obj)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj);

	/* disable the USART TBE interrupt */
	usart_interrupt_disable(p_obj->uart, USART_INT_TBE);

	/* copy sent data from buffer */
	uint32_t i = 0U;
	for (i = 0U; i < p_obj->tx_param.data_length; i++) {
		usart_data_send(p_obj->uart, p_obj->tx_param.prxtx_buff[i]);
	}

	p_obj->tx_state = OP_STATE_READY;
	p_obj->tx_callback(obj);
}


/**
 * @brief   error callback from uart
 * @param   serial obj_s serial object
 * @retval  none
 */
void uart_error_callback(serial_t *obj_s)
{
	struct serial_s *p_obj = GET_SERIAL_S(obj_s);
	uint32_t err_flag = usart_interrupt_flag_get(p_obj->uart);

	usart_interrupt_flag_clear(p_obj->uart, USART_INT_FLAG_ORERR);
	usart_interrupt_flag_clear(p_obj->uart, USART_INT_FLAG_PERR);
	usart_interrupt_flag_clear(p_obj->uart, USART_INT_FLAG_FERR);
	usart_interrupt_flag_clear(p_obj->uart, USART_INT_FLAG_NERR);

	if ((err_flag & USART_INT_FLAG_ORERR) != RESET) {
		p_obj->err_state |= USART_ERROR_ORE;
	}
	if ((err_flag & USART_INT_FLAG_PERR) != RESET) {
		p_obj->err_state |= USART_ERROR_PE;
	}
	if ((err_flag & USART_INT_FLAG_FERR) != RESET) {
		p_obj->err_state |= USART_ERROR_FE;
	}
	if ((err_flag & USART_INT_FLAG_NERR) != RESET) {
		p_obj->err_state |= USART_ERROR_NE;
	}
}

void UART_IRQHandler(struct serial_s *obj_s)
{
	uint32_t isr_flags = usart_interrupt_flag_get(obj_s->uart);

	/* check if we have received data */
	if ((isr_flags & USART_INT_FLAG_RBNE) != RESET) {
		if ((USART_CTL0(obj_s->uart) & USART_CTL0_RBNEIE) != RESET) {
			serial_receive_nb(obj_s, &(obj_s->receiver), 1);
		}
	}

	/* check if we have transmit data */
	if ((isr_flags & USART_INT_FLAG_TBE) != RESET) {
		if ((USART_CTL0(obj_s->uart) & USART_CTL0_TBEIE) != RESET) {
			serial_tx_complete_cb(obj_s);
		}
	}

	/* check if we have error */
	if ((isr_flags & USART_INT_FLAG_ORERR) != RESET) {
		uart_error_callback(obj_s);

	} else if ((isr_flags & USART_INT_FLAG_PERR) != RESET) {
		uart_error_callback(obj_s);

	} else if ((isr_flags & USART_INT_FLAG_FERR) != RESET) {
		uart_error_callback(obj_s);

	} else if ((isr_flags & USART_INT_FLAG_NERR) != RESET) {
		uart_error_callback(obj_s);
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
	UART_IRQHandler(uart_handler_buf[UART0_INDEX]);
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
  if (uart_handler_buf[UART1_INDEX] != NULL) {
    UART_IRQHandler(uart_handler_buf[UART1_INDEX]);
  }
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
  if (uart_handler_buf[UART2_INDEX] != NULL) {
    UART_IRQHandler(uart_handler_buf[UART2_INDEX]);
  }
}
#endif

/**
 * This function handles UART3 interrupt handler
 */
#if defined(UART3)
void UART3_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART3_INDEX]);
	UART_IRQHandler(uart_handler_buf[UART3_INDEX]);
}
#endif

#if defined(USART3)
void USART3_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART3_INDEX]);
	UART_IRQHandler(uart_handler_buf[UART3_INDEX]);
}
#endif

/**
 * This function handles UART4 interrupt handler
 */
#if defined(UART4)
void UART4_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART4_INDEX]);
	UART_IRQHandler(uart_handler_buf[UART4_INDEX]);
}
#endif

#if defined(USART4)
void USART4_IRQHandler(void)
{
	/* clear pending IRQ */
	NVIC_ClearPendingIRQ(usart_irq_n[UART4_INDEX]);
	UART_IRQHandler(uart_handler_buf[UART4_INDEX]);
}
#endif

#ifdef __cplusplus
}
#endif
