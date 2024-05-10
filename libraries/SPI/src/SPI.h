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

#include "Arduino.h"
#include <stdio.h>

extern "C" {
#include "utility/drv_spi.h"
}

/*
 * SPI_HAS_TRANSACTION means SPI has
 *  - beginTransaction()
 *  - endTransaction()
 *  - SPISetting(clock, bitorder, datamode)
 */
#define SPI_HAS_TRANSACTION 1

enum SPITransferMode {
  SPI_CONTINUE, /* Transfer not finished: CS pin kept active */
  SPI_LAST      /* Transfer ended: CS pin released */
};

class SPISettings
{
  public:
    SPISettings(uint32_t speedMax, BitOrder bitOrder, uint8_t dataMode, bool noRX = 0)
    {
      speed = speedMax;
      bitorder = bitOrder;
      datamode = dataMode;
    }

    /* Set speed to default, SPI mode set to MODE 0 and Bit order set to MSB first. */
    SPISettings()
    {
      speed = SPI_SPEED_DEFAULT;
      bitorder = MSBFIRST;
      datamode = SPI_MODE0;
    }

  private:
    uint32_t speed;
    uint8_t datamode;
    BitOrder bitorder;
    int spimode

    friend class SPIClass;
    bool _noRX
};

//const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

class SPIClass
{
  public:
    SPIClass();
    SPIClass(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel = (uint8_t)NC);

    void setMISO(uint32_t miso)
    {
      _spi.pin_miso = DIGITAL_TO_PINNAME(miso);
    };
    void setMOSI(uint32_t mosi)
    {
      _spi.pim_mosi = DIGITAL_TO_PINNAME(mosi);
    };
    void setSCLK(uint32_t sclk)
    {
      _spi.pin_sclk = DIGITAL_TO_PINNAME(sclk);
    };
    void setSSEL(uint32_t ssel)
    {
      _spi.pin_ssel = DIGITAL_TO_PINNAME(ssel);
    };

    void setMISO(PinName miso)
    {
      _spi.pin_miso = (miso);
    };
    void setMOSI(PinName mosi)
    {
      _spi.pin_mosi = (mosi);
    };
    void setSCLK(PinName sclk)
    {
      _spi.pin_sclk = (sclk);
    };
    void setSSEL(PinName ssel)
    {
      _spi.pin_ssel = (ssel);
    };

    void begin();
    void end();

    void beginTransaction(SPISettings settings);
    void endTransaction(void);

    byte transfer(uint8_t data, SPITransferMode mode = SPI_LAST);
    uint16_t transfer16(uint16_t data, SPITransferMode mode = SPI_LAST);
    void transfer(void *buf, size_t count, SPITransferMode mode = SPI_LAST);
    void transfer(void *bufout, void *bufin, size_t count, SPITransferMode mode = SPI_LAST);

    void setBitOrder(BitOrder order);
    void setDataMode(uint8_t mode);
    void setClockDivider(uint32_t divider);

  private:
    SPISettings spiSettings[4];
    int16_t _initialized;
    spi_t _spi;
};

extern SPIClass SPI;

#endif