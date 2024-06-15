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

#define CS_PIN_CONTROLLED_BY_USER   DIGITAL_PINS_NUM
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
      pinCS = -1;
      speed = SPI_SPEED_DEFAULT;
      bitorder = MSBFIRST;
      datamode = SPI_MODE0;
    }

  private:
    int16_t pinCS;
    uint32_t speed;
    uint8_t datamode;
    BitOrder bitorder;
    int spimode;

    friend class SPIClass;
    bool _noRX;
};

//const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

class SPIClass
{
  public:
    SPIClass();
    SPIClass(uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel = PIN_NOT_DEFINED);

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

    void begin(uint8_t pin = CS_PIN_CONTROLLED_BY_USER);
    void end(void);

    void beginTransaction(uint8_t pin, SPISettings settings);
    void beginTransaction(SPISettings settings)
    {
      beginTransaction(CS_PIN_CONTROLLED_BY_USER, settings);
    }

    void endTransaction(uint8_t pin);
    void endTransaction(void)
    {
      endTransaction(CS_PIN_CONTROLLED_BY_USER);
    }

    byte transfer(uint8_t pin, uint8_t data, SPITransferMode mode = SPI_LAST);
    byte transfer(uint8_t data, SPITransferMode mode = SPI_LAST)
    {
      return transfer(CS_PIN_CONTROLLED_BY_USER, data, mode);
    }

    uint16_t transfer16(uint8_t pin, uint16_t data, SPITransferMode mode = SPI_LAST);
    uint16_t transfer16(uint16_t data, SPITransferMode mode = SPI_LAST)
    {
      return transfer16(CS_PIN_CONTROLLED_BY_USER, data, mode);
    }

    void transfer(uint8_t pin, void *buf, size_t count, SPITransferMode mode = SPI_LAST);
    void transfer(void *buf, size_t count, SPITransferMode mode = SPI_LAST)
    {
      transfer(CS_PIN_CONTROLLED_BY_USER, buf, count, mode);
    }

    void transfer(byte pin, void *bufout, void *bufin, size_t count, SPITransferMode mode = SPI_LAST);
    void transfer(void *bufout, void *bufin, size_t count, SPITransferMode mode = SPI_LAST)
    {
      transfer(CS_PIN_CONTROLLED_BY_USER, bufout, bufin, count, mode);
    }

    void setBitOrder(uint8_t pin, BitOrder order);
    void setBitOrder(BitOrder order)
    {
      setBitOrder(CS_PIN_CONTROLLED_BY_USER, order);
    }

    void setDataMode(uint8_t pin, uint8_t mode);
    void setDataMode(uint8_t mode)
    {
      setDataMode(CS_PIN_CONTROLLED_BY_USER, mode);
    }

    void setClockDivider(uint8_t pin, uint32_t divider);
    void setClockDivider(uint32_t divider)
    {
      setClockDivider(CS_PIN_CONTROLLED_BY_USER, divider);
    }

  private:
    SPISettings spiSettings[SPI_SETTINGS_MAX];
    int16_t _CSPinConfig;
    int16_t _initialized;
    spi_t _spi;

    typedef enum {
      GET_IDX = 0,
      ADD_NEW_PIN = 1
    } pin_option_t;

    uint8_t pinIdx(uint8_t pin, pin_option_t option)
    {
      uint8_t i;

      if (pin > DIGITAL_PINS_NUM) {
        return SPI_SETTINGS_MAX;
      }

      for (i = 0; i < SPI_SETTINGS_MAX; i++) {
        if (pin == spiSettings[i].pinCS) {
          return i;
        }
      }

      if (option == ADD_NEW_PIN) {
        for (i = 0; i < SPI_SETTINGS_MAX; i++) {
          if (spiSettings[i].pinCS == -1) {
            spiSettings[i].pinCS = pin;
            return i;
          }
        }
      }
      return i;
    }

    void RemovePin(uint8_t pin)
    {
      if (pin > DIGITAL_PINS_NUM) {
        return;
      }

      for (uint8_t i = 0; i < SPI_SETTINGS_MAX; i++) {
        if (spiSettings[i].pinCS == pin) {
          spiSettings[i].pinCS = -1;
          spiSettings[i].speed = SPI_SPEED_DEFAULT;
          spiSettings[i].bitorder = MSBFIRST;
          spiSettings[i].datamode = SPI_MODE0;
        }
      }
    }

    void RemoveAllPin(void)
    {
      for (uint8_t i = 0; i < SPI_SETTINGS_MAX; i++) {
        spiSettings[i].pinCS = -1;
        spiSettings[i].speed = SPI_SPEED_DEFAULT;
        spiSettings[i].bitorder = MSBFIRST;
        spiSettings[i].datamode = SPI_MODE0;
      }
    }
};

extern SPIClass SPI;

#endif