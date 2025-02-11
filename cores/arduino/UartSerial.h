/*
	UartSerial.h - Hardware serial library for Wiring
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

	Modified 28 September 2010 by Mark Sproul
	Modified 14 August 2012 by Alarus
	Modified 3 December 2013 by Matthijs Kooijman
*/

#ifndef UartSerial_h
#define UartSerial_h

#include "api/HardwareSerial.h"
#include "uart.h"

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
// NOTE: a "power of 2" buffer size is reccomended to dramatically
//       optimize all the modulo operations for ring buffers.
// WARNING: When buffer sizes are increased to > 256, the buffer index
// variables are automatically increased in size, but the extra
// atomicity guards needed for that are not implemented. This will
// often work, but occasionally a race condition can occur that makes
// Serial behave erratically. See https://github.com/arduino/Arduino/issues/2405
#if !defined(SERIAL_TX_BUFFER_SIZE)
#define SERIAL_TX_BUFFER_SIZE 128
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
#define SERIAL_RX_BUFFER_SIZE 128
#endif
#if (SERIAL_TX_BUFFER_SIZE > 256)
typedef uint16_t tx_buffer_index_t;
#else
typedef uint8_t tx_buffer_index_t;
#endif
#if  (SERIAL_RX_BUFFER_SIZE > 256)
typedef uint16_t rx_buffer_index_t;
#else
typedef uint8_t rx_buffer_index_t;
#endif

typedef enum {
	UART_HALFDUPLEX_DISABLE,
	UART_HALFDUPLEX_ENABLE
} uart_halfduplex_flag;

class UartSerial : public arduino::HardwareSerial
{
	protected:
		// for checking if any byte been written to the UART since begin()
		bool _written;

		// Do not put any members after these buffers, since only the first
		// 32 bytes of this struct can be accessed quickly
		unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
		unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];

		serial_t _serial;

	public:
		UartSerial(uint32_t rx, uint32_t tx);
		UartSerial(PinName rx, PinName tx);
		UartSerial(UARTName periph, uart_halfduplex_flag hdFlag = UART_HALFDUPLEX_DISABLE);
		UartSerial(uint32_t rxtx);
		UartSerial(PinName rxtx);
		void begin(unsigned long baud)
		{
			begin(baud, SERIAL_8N1);
		}
		void begin(unsigned long baud, uint16_t config);
		void end(void);
		int available(void);
		int peek(void);
		int read(void);
		void flush(void);
		size_t write(uint8_t);
		inline size_t write(unsigned long n)
		{
			return write((uint8_t)n);
		}
		inline size_t write(long n)
		{
			return write((uint8_t)n);
		}
		inline size_t write(unsigned int n)
		{
			return write((uint8_t)n);
		}
		inline size_t write(int n)
		{
			return write((uint8_t)n);
		}
		using Print::write;
		operator bool()
		{
			return true;
		}

		int availableForWrite(void);

		// enable halfduplex mode by setting rx pin to NC
		// needs to be done before the call to begin()
		void setHalfDuplex(void);
		bool isHalfDuplex(void) const;
		void enableHalfDuplexRx(void);
		void setRx(uint32_t rx);
		void setTx(uint32_t tx);
		void setRx(PinName rx);
		void setTx(PinName tx);
		static void _rx_complete_irq(serial_t *obj);
		static int _tx_complete_irq(serial_t *obj);

	private:
		bool _rx_enabled;
		uint16_t _config;
		unsigned long _baud;
		void init(PinName rx, PinName tx);
};

extern UartSerial Serial1;
extern UartSerial Serial2;
extern UartSerial Serial3;
extern UartSerial Serial4;
extern UartSerial Serial5;

	//protected:
		// for checking if any byte been written to the UART since begin()
		//bool _written;

		// Do not put any members after these buffers, since only the first
		// 32 bytes of this struct can be accessed quickly
		//unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
		//unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];

		//serial_t _serial;

	//public:
		//HardwareSerial(uint32_t rx, uint32_t tx);
		//HardwareSerial(PinName rx, PinName tx);
		//HardwareSerial(UARTName periph, uart_halfduplex_flag hdFlag = UART_HALFDUPLEX_DISABLE);
		//HardwareSerial(uint32_t rxtx);
		//HardwareSerial(PinName rxtx);
		//void begin(unsigned long baud)
		//{
		//	begin(baud, SERIAL_8N1);
		//}
		//void begin(unsigned long baud, uint8_t config);
		//void end();
		//virtual int available(void);
		//virtual int peek(void);
		//virtual int read(void);
		//virtual void flush(void);
		//virtual size_t write(uint8_t);
		//inline size_t write(unsigned long n)
		//{
		//	return write((uint8_t)n);
		//}
		//inline size_t write(long n)
		//{
		//	return write((uint8_t)n);
		//}
		//inline size_t write(unsigned int n)
		//{
		//	return write((uint8_t)n);
		//}
		//inline size_t write(int n)
		//{
		//	return write((uint8_t)n);
		//}
		//using Print::write; // pull in write(str) and write(buf, size) from Print
		//operator bool()
		//{
		//	return true;
		//}
		//int availableForWrite(void);
		// enable halfduplex mode by setting rx pin to NC
		// needs to be done before the call to begin()
		//void setHalfDuplex(void);
		//bool isHalfDuplex(void) const;
		//void enableHalfDuplexRx(void);
		//void setRx(uint32_t rx);
		//void setTx(uint32_t tx);
		//void setRx(PinName rx);
		//void setTx(PinName tx);
		// interrupt handlers
		//static void _rx_complete_irq(serial_t *obj);
		//static int _tx_complete_irq(serial_t *obj);

	//private:
		//bool _rx_enabled;
		//uint8_t _config;
		//unsigned long _baud;
		//void init(PinName rx, PinName tx);
//};

//extern HardwareSerial Serial1;
//extern HardwareSerial Serial2;
//extern HardwareSerial Serial3;
//extern HardwareSerial Serial4;
//extern HardwareSerial Serial5;

#endif  /* UartSerial_h */
