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
#include "api/HardwareSPI.h"

#include <stdio.h>

extern "C" {
#include "utility/drv_spi.h"
}

//#define CS_PIN_CONTROLLED_BY_USER   DIGITAL_PINS_NUM
#define NO_CONFIG   ((int16_t)(-1))
/*
 * SPI_HAS_TRANSACTION means SPI has
 *  - beginTransaction()
 *  - endTransaction()
 *  - SPISetting(clock, bitorder, datamode)
 */
#define SPI_HAS_TRANSACTION 1

#ifndef SPI_SETTINGS_MAX
#define SPI_SETTINGS_MAX    4
#endif

typedef enum {
  SPI_CONTINUE, /* Transfer not finished: CS pin kept active */
  SPI_LAST      /* Transfer ended: CS pin released */
} SPITransferMode;

class SPIClassGD : public arduino::HardwareSPI {
  public:
    SPIClassGD();
    SPIClassGD(uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel = PIN_NOT_DEFINED);

    void setMISO(uint32_t miso)
    {
      _spi.pin_miso = DIGITAL_TO_PINNAME(miso);
    };
    void setMOSI(uint32_t mosi)
    {
      _spi.pin_mosi = DIGITAL_TO_PINNAME(mosi);
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

    void begin(void);
    void end(void);

    void beginTransaction(SPISettings settings);

    void endTransaction(void);

    uint8_t transfer(uint8_t data, SPITransferMode mode = SPI_LAST);
    uint8_t transfer(uint8_t data)
    {
      transfer(data, SPI_LAST);
    }

    uint16_t transfer16(uint16_t data, SPITransferMode mode = SPI_LAST);
    uint16_t transfer16(uint16_t data)
    {
      return transfer16(data, SPI_LAST);
    }

    void transfer(void *buf, size_t count, SPITransferMode mode = SPI_LAST);
    void transfer(void *buf, size_t count)
    {
      transfer(buf, count, SPI_LAST);
    }

    void transfer(void *bufout, void *bufin, size_t count, SPITransferMode mode = SPI_LAST);
    void transfer(void *bufout, void *bufin, size_t count)
    {
      transfer(bufout, bufin, count, SPI_LAST);
    }

    void setBitOrder(BitOrder order);
    void setDataMode(uint8_t mode);
    void setClockDivider(uint32_t divider);

  private:
    void config(SPISettings spiSettings);

    SPISettings spiSettings[SPI_SETTINGS_MAX];
    bool _initialized;
    spi_t _spi;
};

extern SPIClassGD SPI;

#ifdef SPI1
  extern SPICllassGD SPI1;
#endif
#ifdef SPI2
  extern SPIClassGD SPI2;
#endif

#endif