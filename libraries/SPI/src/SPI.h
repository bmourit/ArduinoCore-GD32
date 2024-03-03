/*
 * SPI Master library
 * Copyright (c) 2015 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Modified by Ajay Bhargav <contact@rickeyworld.info>
 * Modified by Gigadevice
 *
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>
#include <api/HardwareSPI>
#include <stdio.h>

extern "C" {
  #include "utility/drv_spi.h"
}

class SPIClassGD32 : public arduino::HardwareSPI 
{
	public:
		SPIClassGD32();
		SPIClassGD32(PinName mosi, PinName miso, PinName sclk, PinName ssel);
		SPIClassGD32(PinName mosi, PinName miso, PinName sclk);

		void begin();
		void end();

		void beginTransaction(SPISettings settings) override;
		void endTransaction(void) override;

		uint8_t transfer(uint8_t data);
		uint16_t transfer16(uint16_t data);
		void transfer(void *buf, size_t count);
		void transfer(void *bufout, void *bufin, size_t count);

		void setBitOrder(BitOrder order);
		void setDataMode(uint8_t mode);
		void setClockDivider(uint32_t divider);

		/* not implemented */
		void usingInterrupt(int interruptNumber);
		void notUsingInterrupt(int interruptNumber);
		void attachInterrupt();
		void detatchInterrupt();

	private:
		void config(SPISettings settings);

		SPISettings spisettings;

		bool initialized;
		spi_t _spi;
};

extern SPIClassGD32 SPI;

#endif /*_SPI_H_INCLUDED */
