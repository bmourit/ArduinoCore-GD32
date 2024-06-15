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

#include "SPI.h"

SPIClass SPI;

SPIClass::SPIClass(void) : _CSPinConfig(NO_CONFIG)
{
  _spi.pin_miso = DIGITAL_TO_PINNAME(MISO);
  _spi.pin_mosi = DIGITAL_TO_PINNAME(MOSI);
  _spi.pin_sclk = DIGITAL_TO_PINNAME(SCK);
  _spi.pin_ssel = NC;
  _initialized = false;
}

SPIClass::SPIClass(uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel) : _CSPinConfig(NO_CONFIG)
{
  _spi.pin_miso = DIGITAL_TO_PINNAME(miso);
  _spi.pin_mosi = DIGITAL_TO_PINNAME(mosi);
  _spi.pin_sclk = DIGITAL_TO_PINNAME(sclk);
  _spi.pin_ssel = DIGITAL_TO_PINNAME(ssel);
  _initialized = false;
}

void SPIClass::begin(uint8_t pin)
{
  if (_initialized) {
    return;
  }
  uint8_t idx;
  if (pin > DIGITAL_PINS_NUM) {
    return;
  }
  idx = pinIdx(pin, ADD_NEW_PIN);
  if (idx >= SPI_SETTINGS_MAX) {
    return;
  }
  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (_spi.pin_ssel == NC)) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }

  spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
  _CSPinConfig = pin;
  _initialized = true;
}

void SPIClass::beginTransaction(uint8_t pin, SPISettings settings)
{
  uint8_t idx;

  if (pin > DIGITAL_PINS_NUM) {
    return;
  }

  idx = pinIdx(pin, ADD_NEW_PIN);
  if (idx > SPI_SETTINGS_MAX) {
    return;
  }

  spiSettings[idx].speed = settings.speed;
  spiSettings[idx].bitorder = settings.bitorder;
  spiSettings[idx].datamode = settings.datamode;
  spiSettings[idx]._noRX = settings._noRX;

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (_spi.pin_ssel == NC)) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }

  spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
  _CSPinConfig = pin;
  _initialized = true;
}

void SPIClass::endTransaction(uint8_t pin)
{
  if (pin > DIGITAL_PINS_NUM) {
    return;
  }

  RemovePin(pin);
  _CSPinConfig = NO_CONFIG;
  _initialized = false;
}

/**
  * @brief  Deinitialize the SPI instance and stop it.
  */
void SPIClass::end()
{
  spi_free(&_spi);
  RemoveAllPin();
  _CSPinConfig = NO_CONFIG;
  _initialized = false;
}

byte SPIClass::transfer(uint8_t pin, uint8_t data, SPITransferMode mode )
{
  uint8_t rx_buffer = 0;

  if (pin > DIGITAL_PINS_NUM) {
    return rx_buffer;
  }

  uint8_t idx = pinIdx(pin, GET_IDX);
  if (pin != _CSPinConfig) {
    if (idx >= SPI_SETTINGS_MAX) {
      return rx_buffer;
    }
    spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
    _CSPinConfig = pin;
  }

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, LOW);
  }

  spi_transfer(&_spi, ((uint8_t *)&data), ((uint8_t *)&rx_buffer), sizeof(uint8_t), spiSettings[idx]._noRX);

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (mode == SPI_LAST) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, HIGH);
  }

  return rx_buffer;
}

uint16_t SPIClass::transfer16(uint8_t pin, uint16_t data, SPITransferMode mode)
{
  uint16_t rx_buffer = 0;
  uint8_t tmp;

  if (pin > DIGITAL_PINS_NUM) {
    return rx_buffer;
  }

  uint8_t idx = pinIdx(pin, GET_IDX);
  if (pin != _CSPinConfig) {
    return rx_buffer;
  }

  if (pin != _CSPinConfig) {
    spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
    _CSPinConfig = pin;
  }

  if (spiSettings[idx].bitorder) {
    tmp = ((data & 0xff00) >> 8) | ((data & 0xff) << 8);
    data = tmp;
  }

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, LOW);
  }

  spi_transfer(&_spi, (uint8_t *)&data, (uint8_t *)&rx_buffer, sizeof(uint16_t), spiSettings[idx]._noRX);

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (mode == SPI_LAST) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, HIGH);
  }

  if (spiSettings[idx].bitorder) {
    tmp = ((rx_buffer & 0xff00) >> 8) | ((rx_buffer & 0xff) << 8);
    rx_buffer = tmp;
  }

  return rx_buffer;
}

void SPIClass::transfer(uint8_t pin, void *buf, size_t count, SPITransferMode mode)
{
  if ((count == 0) || (buf == NULL) || (pin > DIGITAL_PINS_NUM)) {
    return;
  }

  uint8_t idx = pinIdx(pin, GET_IDX);
  if (pin != _CSPinConfig) {
    if (idx >= SPI_SETTINGS_MAX) {
      return;
    }
    spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
    _CSPinConfig = pin;
  }

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, LOW);
  }

  spi_transfer(&_spi, ((uint8_t *)buf), ((uint8_t *)buf), count, spiSettings[idx]._noRX);

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (mode == SPI_LAST) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, HIGH);
  }
}

void SPIClass::transfer(byte pin, void *bufout, void *bufin, size_t count, SPITransferMode mode)
{
  if ((count == 0) || (bufout == NULL) || (bufin == NULL)  || (pin > DIGITAL_PINS_NUM)) {
    return;
  }
  uint8_t idx = pinIdx(pin, GET_IDX);
  if (pin != _CSPinConfig) {
    if (idx >= SPI_SETTINGS_MAX) {
      return;
    }
    spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
    _CSPinConfig = pin;
  }

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, LOW);
  }

  spi_transfer(&_spi, ((uint8_t *)bufout), ((uint8_t *)bufin), count, spiSettings[idx]._noRX);

  if ((pin != CS_PIN_CONTROLLED_BY_USER) && (mode == SPI_LAST) && (_spi.pin_ssel == NC)) {
    digitalWrite(pin, HIGH);
  }
}

void SPIClass::setBitOrder(uint8_t pin, BitOrder order)
{
  if (pin > DIGITAL_PINS_NUM) {
    return;
  }

  uint8_t idx = pinIdx(pin, GET_IDX);
  if (idx >= SPI_SETTINGS_MAX) {
    return;
  }
  spiSettings[idx].bitorder = order;
  spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
}

void SPIClass::setDataMode(uint8_t pin, uint8_t mode)
{
  if (pin > DIGITAL_PINS_NUM) {
    return;
  }

  uint8_t idx = pinIdx(pin, GET_IDX);
  if (idx >= SPI_SETTINGS_MAX) {
    return;
  }

  spiSettings[idx].datamode = mode;
  spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
}

void SPIClass::setClockDivider(uint8_t pin, uint32_t divider)
{
  if (pin > DIGITAL_PINS_NUM) {
    return;
  }

  uint8_t idx = pinIdx(pin, GET_IDX);
  if (idx >= SPI_SETTINGS_MAX) {
    return;
  }

  if (divider == 0) {
    spiSettings[idx].speed = SPI_SPEED_DEFAULT;
  } else {
    /* Get clk freq of the SPI instance and compute it */
    spiSettings[idx].speed = spi_getClkFreq(&_spi) / divider;
  }

  spi_begin(&_spi, spiSettings[idx].speed, spiSettings[idx].datamode, spiSettings[idx].bitorder);
}
