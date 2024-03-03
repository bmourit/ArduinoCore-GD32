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

#include <Arduino.h>
#include <assert.h>

#include "pins_arduino.h"
#include "SPI.h"

SPIClassGD32::SPIClassGD32(void)
{
    _spi.pin_miso = DIGITAL_TO_PINNAME(MISO);
    _spi.pin_mosi = DIGITAL_TO_PINNAME(MOSI);
    _spi.pin_sclk = DIGITAL_TO_PINNAME(SCK);
    _spi.pin_ssel = NC;

    initialized = false;
}

SPIClassGD32::SPIClassGD32(PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    _spi.pin_miso = miso;
    _spi.pin_mosi = mosi;
    _spi.pin_sclk = sclk;
    _spi.pin_ssel = ssel;

    initialized = false;
}

SPIClassGD32::SPIClassGD32(PinName mosi, PinName miso, PinName sclk)
{
    _spi.pin_miso = miso;
    _spi.pin_mosi = mosi;
    _spi.pin_sclk = sclk;
    _spi.pin_ssel = NC;

    initialized = false;
}

void SPIClassGD32::begin()
{
    uint32_t test = 0;

    if (!initialized) {
        spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
    }

    initialized = true;
}

void SPIClassGD32::end()
{
    if (initialized) {
        spi_free(&_spi);
        initialized = false;
    }
}

void SPIClassGD32::beginTransaction(SPISettings settings)
{
    config(settings);
    spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
    initialized = true;
}

void SPIClassGD32::endTransaction(void)
{
    if (initialized) {
        spi_free(&_spi);
        initialized = false;
    }
}

uint8_t SPIClassGD32::transfer(uint8_t val8)
{
    uint32_t out_byte;
    out_byte = spi_master_write(&_spi, val8);

    return out_byte;
}

uint16_t SPIClassGD32::transfer16(uint16_t data)
{
    uint16_t odata;

    if (spisettings.bitorder == MSBFIRST) {
        odata = ((data & 0xff00) >> 8) | ((data & 0xff) << 8);
    } else {
        odata = data;
    }

    transfer((odata & 0xff));
    transfer((odata & 0xff00));

    return odata;
}

void SPIClassGD32::transfer(void *buf, size_t count)
{
    spi_master_block_write(&_spi, ((uint8_t *)buf), ((uint8_t *)buf), count);
}

void SPIClassGD32::transfer(void *bufout, void *bufin, size_t count)
{
    spi_master_block_write(&_spi, ((uint8_t *)bufout), ((uint8_t *)bufin), count);
}

void SPIClassGD32::setBitOrder(BitOrder order)
{
    spisettings.bitorder = order;
    spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
}

void SPIClassGD32::setDataMode(uint8_t mode)
{
    spisettings.datamode = mode;
    spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
}

void SPIClassGD32::setClockDivider(uint32_t divider)
{
    if (divider == 0) {
        spisettings.speed = SPI_SPEED_DEFAULT;
    } else {
        /* Get clk freq of the SPI instance and compute it */
        spisettings.speed = dev_spi_clock_source_frequency_get(&_spi) / divider;
    }

    spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
}

void SPIClassGD32::config(SPISettings settings)
{
    if (this->settings != settings) {
        this->settings = settings;
    }
}

/**
  * @brief  Not implemented.
  */
void SPIClassGD32::usingInterrupt(int interruptNumber)
{
  UNUSED(interruptNumber);
}

/**
  * @brief  Not implemented.
  */
void SPIClassGD32::notUsingInterrupt(int interruptNumber)
{
  UNUSED(interruptNumber);
}

/**
  * @brief  Not implemented.
  */
void SPIClassGD32::attachInterrupt(void)
{
  // Should be enableInterrupt()
}

/**
  * @brief  Not implemented.
  */
void SPIClassGD32::detachInterrupt(void)
{
  // Should be disableInterrupt()
}
