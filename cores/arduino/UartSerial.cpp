/*
  UartSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#include <stdio.h>
#include "Arduino.h"
#include "UartSerial.h"

#if defined(SPL_USART_ENABLE)
#if defined(HAVE_HWSERIAL1) || defined(HAVE_HWSERIAL2) || defined(HAVE_HWSERIAL3) || defined(HAVE_HWSERIAL4) || defined(HAVE_HWSERIAL5)

// SerialEvent functions are weak, so when the user doesn't define them,
// the linker just sets their address to 0 (which is checked below).
#if defined(HAVE_HWSERIAL1)
UartSerial Serial1(UART_0);
void serialEvent1() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL2)
UartSerial Serial2(UART_1);
void serialEvent2() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL3)
UartSerial Serial3(UART_2);
void serialEvent3() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL4)
UartSerial Serial4(UART_3);
void serialEvent4() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL5)
UartSerial Serial5(UART_4);
void serialEvent5() __attribute__((weak));
#endif
#endif

//#define SERIAL_UART_DATA_MASK   0x07

UartSerial::UartSerial(uint32_t rx, uint32_t tx)
{
  init(DIGITAL_TO_PINNAME(rx), DIGITAL_TO_PINNAME(tx));
}

UartSerial::UartSerial(PinName rx, PinName tx)
{
  init(rx, tx);
}

UartSerial::UartSerial(UARTName periph, uart_halfduplex_flag hdFlag)
{
  /* assume half-duplex if pin_rx is not defined */
  _serial.pin_rx = NC;
  _serial.pin_tx = NC;
  /* set the rx/tx pins if Serial is defined in variant */
#if defined(Serial) && defined(PIN_SERIAL_TX)
  if (this == &Serial) {
#if defined(PIN_SERIAL_RX)
    setRx(PIN_SERIAL_RX);
#endif
    setTx(PIN_SERIAL_TX);
  } else
#endif
#if defined(PIN_SERIAL1_TX) && defined(USART0)
  if (periph == UART_0) {
#if defined(PIN_SERIAL1_RX)
    serRx(PIN_SERIAL1_RX);
#endif
    setTx(PIN_SERIAL1_TX);
  } else
#endif
#if defined(PIN_SERIAL2_TX) && defined(USART1)
  if (periph == UART_1) {
#if defined(PIN_SERIAL2_RX)
    serRx(PIN_SERIAL2_RX);
#endif
    setTx(PIN_SERIAL2_TX);
  } else
#endif
#if defined(PIN_SERIAL3_TX) && defined(USART2)
  if (periph == UART_2) {
#if defined(PIN_SERIAL3_RX)
    serRx(PIN_SERIAL3_RX);
#endif
    setTx(PIN_SERIAL3_TX);
  } else
#endif
#if defined(PIN_SERIAL4_TX) && (defined(UART3) || defined(USART3))
  if (periph == UART_3) {
#if defined(PIN_SERIAL4_RX)
    serRx(PIN_SERIAL4_RX);
#endif
    setTx(PIN_SERIAL4_TX);
  } else
#endif
#if defined(PIN_SERIAL5_TX) && (defined(UART4) || defined(USART4))
  if (periph == UART_4) {
#if defined(PIN_SERIAL5_RX)
    serRx(PIN_SERIAL5_RX);
#endif
    setTx(PIN_SERIAL5_TX);
  } else
#endif
  {
    _serial.pin_rx = pinmap_pin((UARTName)periph, PinMap_UART_RX);
    _serial.pin_tx = pinmap_pin((UARTName)periph, PinMap_UART_TX);
  }

  // Handle half-duplex mode
  if (hdFlag == UART_HALFDUPLEX_ENABLE) {
    _serial.pin_rx = NC;
  }

  init(_serial.pin_rx, _serial.pin_tx);
}

UartSerial::UartSerial(uint32_t rxtx)
{
  init(NC, DIGITAL_TO_PINNAME(rxtx));
}

UartSerial::UartSerial(PinName rxtx)
{
  init(NC, rxtx);
}

void UartSerial::init(PinName rx, PinName tx)
{
  _serial.pin_rx = (rx == tx) ? NC : rx;
  _serial.pin_tx = tx;
  _serial.rx_buff = _rx_buffer;
  _serial.rx_head = 0;
  _serial.rx_tail = 0;
  _serial.tx_buff = _tx_buffer;
  _serial.tx_head = 0;
  _serial.tx_tail = 0;
}

void UartSerial::_rx_complete_irq(serial_t *obj)
{
  unsigned char c;
  if (serial_getc(obj, &c) == 0) {
    rx_buffer_index_t i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;
    if (i != obj->rx_tail) {
      obj->rx_buff[obj->rx_head] = c;
      obj->rx_head = i;
    }
  }
}

int UartSerial::_tx_complete_irq(serial_t *obj)
{
  obj->tx_tail = (obj->tx_tail + 1) % SERIAL_TX_BUFFER_SIZE;
  if (obj->tx_head == obj->tx_tail) {
    return -1;
  }

  return 0;
}

void UartSerial::begin(unsigned long baud, uint16_t config)
{
  uint32_t databits = 0;
  uint32_t stopbits = 0;
  SerialParity parity;

  _baud = baud;
  _config = config;

  // Manage databits
  switch (config & SERIAL_DATA_MASK) {
    //case SERIAL_DATA_6:
      //databits = USART_WL_6BIT;
      //break;
    //case SERIAL_DATA_7:
      //databits = USART_WL_7BIT;
      //break;
    case SERIAL_DATA_8:
      databits = USART_WL_8BIT;
      break;
    default:
      databits = 0;
      break;
  }

  // Manage parity
  if ((config & SERIAL_PARITY_MASK) == SERIAL_PARITY_ODD) {
    parity = ParityOdd;
    databits++; // Increase databits for parity
  } else if ((config & SERIAL_PARITY_MASK) == SERIAL_PARITY_EVEN) {
    parity = ParityEven;
    databits++; // Increase databits for parity
  } else {
    parity = ParityNone;
  }

  // Manage stopbits
  if ((config & SERIAL_STOP_BIT_MASK) == SERIAL_STOP_BIT_2) {
    stopbits = USART_STB_2BIT;
  } else {
    stopbits = USART_STB_1BIT;
  }

  // Adjust databits
  switch (databits) {
    case 8:
      databits = USART_WL_8BIT;
      break;
    case 9:
      databits = USART_WL_9BIT;
      break;
    default:
      // Handle invalid databits configuration
      Error_Handler();
      break;
  }

  // Initialize the serial peripheral
  serial_init(&_serial, (uint32_t)baud, databits, (SerialParity)parity, stopbits);

  // Enable half-duplex RX if needed
  if (_serial.pin_rx == NC) {
    enableHalfDuplexRx();
  }

  // Attach RX complete IRQ handler
  uart_attach_rx_callback(&_serial, _rx_complete_irq);
}

void UartSerial::end()
{
  // Wait for any outstanding data to be sent
  flush();
  // Disable and deinit the USART
  serial_free(&_serial);
  // Clear received data
  _serial.rx_head = _serial.rx_tail;
}

int UartSerial::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _serial.rx_head - _serial.rx_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int UartSerial::peek(void)
{
  if (_serial.rx_head  == _serial.rx_tail) {
    return -1;
  } else {
    return _serial.rx_buff[_serial.rx_tail];
  }
}

int UartSerial::read(void)
{
  enableHalfDuplexRx();
  // we don't have any characters if the head isn't ahead of the tail
  if (_serial.rx_head == _serial.rx_tail) {
    return -1;
  } else {
    unsigned char c = _serial.rx_buff[_serial.rx_tail];
    _serial.rx_tail = (rx_buffer_index_t)(_serial.rx_tail + 1) % SERIAL_RX_BUFFER_SIZE;
    return c;
  }
}

int UartSerial::availableForWrite(void)
{
  tx_buffer_index_t head = _serial.tx_head;
  tx_buffer_index_t tail = _serial.tx_tail;

  if (head >= tail) {
    return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
  }

  return tail - head - 1;
}

void UartSerial::flush()
{
  // no need to flush if we have never written a byte. This special
  // case is needed since there is no way to force the TC (transmit
  // complete) bit to 1 during initialization
  if (!_written) {
    return;
  }

  while ((_serial.tx_head != _serial.tx_tail)) {
    // nop, since the interrupt handler will free the space
  }
  // nothing is queued anymore if we get here
}

size_t UartSerial::write(uint8_t c)
{
  _written = true;
  if (isHalfDuplex()) {
    if (_rx_enabled) {
      _rx_enabled = false;
      serial_enable_rx(&_serial);
    }
  }

  tx_buffer_index_t nextWrite = (_serial.tx_head + 1) % SERIAL_TX_BUFFER_SIZE;
  while (nextWrite == _serial.tx_tail) {
    // spin locks if we're about to overwrite the buffer. This continues once the data is sent
  }
  _serial.tx_buff[_serial.tx_head] = c;
  _serial.tx_head = nextWrite;

  if (!serial_tx_active(&_serial)) {
    uart_attach_tx_callback(&_serial, _tx_complete_irq);
  }

  return 1;
}

void UartSerial::setRx(uint32_t rx)
{
  _serial.pin_rx = DIGITAL_TO_PINNAME(rx);
}

void UartSerial::setTx(uint32_t tx)
{
  _serial.pin_tx = DIGITAL_TO_PINNAME(tx);
}

void UartSerial::setRx(PinName rx)
{
  _serial.pin_rx = rx;
}

void UartSerial::setTx(PinName tx)
{
  _serial.pin_tx = tx;
}

void UartSerial::setHalfDuplex(void)
{
  _serial.pin_rx = NC;
}

bool UartSerial::isHalfDuplex(void) const
{
  return _serial.pin_rx == NC;
}

void UartSerial::enableHalfDuplexRx(void)
{
  if (isHalfDuplex()) {
    // In half-duplex mode we have to wait for all TX characters to
    // be transmitted before we can receive data.
    flush();
    if (!_rx_enabled) {
      _rx_enabled = true;
      serial_enable_rx(&_serial);
    }
  }
}

#endif /* SPL_USART_ENABLE */
