/*
  Copyright (c) 2020, GigaDevice Semiconductor Inc.

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

  Based on mbed-os/target/TARGET_GigaDevice/TARGET_GD32F30X/spi_api.c
*/

#include "gd_debug.h"
#include "gd32_def.h"
#include "utility/drv_spi.h"
#include "gd32f30x_remap.h"
//#include "gd32xxyy_spl_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_PINS_FREE_MODE   0x00000001

/**
 * Initialize the SPI structure
 * Configures the pins used by SPI, sets a default format and frequency, and enables the peripheral
 * @param[out] spi_handle  The SPI handle to initialize
 */
void spi_set_config(SPL_SPIHandle_t *spi_handle)
{
  if (spi_handle == NULL) {
    return;
  }

  spi_disable(spi_handle->instance);
  spi_init(spi_handle->instance, &spi_handle->params);
  spi_enable(spi_handle->instance);
}

/**
 * @brief   get the clock frequency of the SPI instance
 * @param   spi_instance : the spi instance name
 * @retval  returns the clock freq of the SPI instance else SystemCoreClock
 */
uint32_t spi_getClkFreqInstance(SPIName spi_instance)
{
  uint32_t spi_freq = SystemCoreClock;

  if (spi_instance != (SPIName)NP) {
    switch (spi_instance) {
      case SPI_0:
        /* clock source is APB2 */
        spi_freq = rcu_clock_freq_get(CK_APB2);
        break;
#if defined(SPI1) || defined(SPI2)
#if defined(SPI1)
      case SPI_1:
#endif
#ifdef SPI2
      case SPI_2:
#endif
        /* clock source is APB1 */
        spi_freq = rcu_clock_freq_get(CK_APB1);
        break;
#endif
      default:
        gd_debug("SPI Clock: SPI instance not set");
        break;
    }
  }

  return spi_freq;
}

/**
 * @brief   get the clock frequency of SPI instance
 * @param   obj : pointer to spi structure
 * @retval  returns the clock freq of the SPI instance else SystemCoreClock
 */
uint32_t spi_getClkFreq(spi_t *obj)
{
  SPIName spi_instance = NP;
  uint32_t spi_freq = SystemCoreClock;

  if (obj != NULL) {
    spi_instance = pinmap_peripheral(obj->pin_sclk, PinMap_SPI_SCLK);

    if (spi_instance != NP) {
      spi_freq = spi_getClkFreqInstance(spi_instance);
    }
  }
  return spi_freq;
}

/**
  * @brief  SPI initialization function
  * @param  obj : pointer to spi_t structure
  * @param  speed : spi output speed
  * @param  mode : one of the spi modes
  * @param  endian : set to 1 in msb first
  * @retval None
  */
void spi_begin(spi_t *obj, uint32_t speed, uint8_t mode, uint8_t endian)
{
  if (obj == NULL) {
    return;
  }

  SPL_SPIHandle_t *handle = &(obj->handle);
  uint32_t spi_freq = 0;
  uint32_t pull = 0;

  /* Determine the SPI to use */
  SPIName spi_mosi = (SPIName)pinmap_peripheral(obj->pin_mosi, PinMap_SPI_MOSI);
  SPIName spi_miso = (SPIName)pinmap_peripheral(obj->pin_miso, PinMap_SPI_MISO);
  SPIName spi_sclk = (SPIName)pinmap_peripheral(obj->pin_sclk, PinMap_SPI_SCLK);
  SPIName spi_ssel = (SPIName)pinmap_peripheral(obj->pin_ssel, PinMap_SPI_SSEL);

  /* only ssel can be NP */
  if (spi_mosi == NP || spi_miso == NP || spi_sclk == NP) {
    gd_debug("ERROR: at least one SPI pin has no peripheral\n");
    return;
  }

  /* return SPIName according to PinName */
  SPIName spi_data = (SPIName)pinmap_merge(spi_mosi, spi_miso);
  SPIName spi_cntl = (SPIName)pinmap_merge(spi_sclk, spi_ssel);

  obj->spi = (SPIName)pinmap_merge(spi_data, spi_cntl);

  if (spi_data == NP || spi_cntl == NP || obj->spi == NP) {
    gd_debug("ERROR: mismatching SPI pins\n");
    return;
  }

  if (obj->pin_ssel != NC) {
    handle->params.nss = SPI_NSS_HARD;
  } else {
    handle->params.nss = SPI_NSS_SOFT;
  }

  handle->instance = obj->spi;
  handle->params.device_mode = SPI_MASTER;

  spi_freq = spi_getClkFreqInstance(obj->spi);
  if (speed >= (spi_freq / SPI_CLOCK_DIV2)) {
    handle->params.prescale = SPI_PSC_2;
  } else if (speed >= (spi_freq / SPI_CLOCK_DIV4)) {
    handle->params.prescale = SPI_PSC_4;
  } else if (speed >= (spi_freq / SPI_CLOCK_DIV8)) {
    handle->params.prescale = SPI_PSC_8;
  } else if (speed >= (spi_freq / SPI_CLOCK_DIV16)) {
    handle->params.prescale = SPI_PSC_16;
  } else if (speed >= (spi_freq / SPI_CLOCK_DIV32)) {
    handle->params.prescale = SPI_PSC_32;
  } else if (speed >= (spi_freq / SPI_CLOCK_DIV64)) {
    handle->params.prescale = SPI_PSC_64;
  } else if (speed >= (spi_freq / SPI_CLOCK_DIV128)) {
    handle->params.prescale = SPI_PSC_128;
  } else {
    /*
     * As it is not possible to go below (spi_freq / SPI_SPEED_CLOCK_DIV256_MHZ).
     * Set prescaler at max value so get the lowest frequency possible.
     */
    handle->params.prescale = SPI_PSC_256;
  }

  handle->params.trans_mode = SPI_TRANSMODE_FULLDUPLEX;

  if (mode == SPI_MODE0) {
    handle->params.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
  } else if (mode == SPI_MODE1) {
    handle->params.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
  } else if (mode == SPI_MODE2) {
    handle->params.clock_polarity_phase =  SPI_CK_PL_HIGH_PH_1EDGE;
  } else if (mode == SPI_MODE3) {
    handle->params.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
  }

  handle->params.frame_size = SPI_FRAMESIZE_8BIT;

  if (endian == 0) {
    handle->params.endian = SPI_ENDIAN_LSB;
  } else {
    handle->params.endian = SPI_ENDIAN_MSB;
  }

  /* configure GPIO mode of SPI pins */
  pinmap_pinout(obj->pin_mosi, PinMap_SPI_MOSI);
  pinmap_pinout(obj->pin_miso, PinMap_SPI_MISO);
  pinmap_pinout(obj->pin_sclk, PinMap_SPI_SCLK);

  /* PULLUP or PULLDOWN SCLK pin according to polarity */
  switch (handle->params.clock_polarity_phase) {
    case SPI_CK_PL_LOW_PH_1EDGE:
    case SPI_CK_PL_LOW_PH_2EDGE:
      pull = GPIO_PULLDOWN;
      break;
    case SPI_CK_PL_HIGH_PH_1EDGE:
    case SPI_CK_PL_HIGH_PH_2EDGE:
      pull = GPIO_PULLUP;
      break;
    default:
      pull = GPIO_NOPULL;
      break;
  }
  f3_pin_pull_config(APORT_TO_GPORT(GD_PORT_GET(obj->pin_sclk)), APIN_TO_GPIN(GD_PIN_GET(obj->pin_sclk)), pull);
  pinmap_pinout(obj->pin_ssel, PinMap_SPI_SSEL);

  /* enable SPI clock */
  if (handle->instance == SPI0) {
    rcu_periph_clock_enable(RCU_SPI0);
  }
#ifdef SPI1
  if (handle->instance == SPI1) {
    rcu_periph_clock_enable(RCU_SPI1);
  }
#endif
#ifdef SPI2    
  if (handle->instance == SPI2) {
    rcu_periph_clock_enable(RCU_SPI2);
  }
#endif

  spi_set_config(handle);
  spi_enable(handle->instance);
}

/**
  * @brief This function is implemented to deinitialize the SPI interface
  *        (IOs + SPI block)
  * @param  obj : pointer to spi_t structure
  * @retval None
  */
void spi_free(spi_t *obj)
{
  if (obj == NULL) {
    return;
  }

  SPL_SPIHandle_t *handle = &(obj->handle);

  spi_disable(handle->instance);

  /* Disable and deinit SPI */
  if (handle->instance == SPI0) {
    spi_i2s_deinit(SPI0);
    rcu_periph_clock_disable(RCU_SPI0);
  }
  if (handle->instance == SPI1) {
    spi_i2s_deinit(SPI1);
    rcu_periph_clock_disable(RCU_SPI1);
  }
#ifdef SPI2
  if (handle->instance == SPI2) {
    spi_i2s_deinit(SPI2);
    rcu_periph_clock_disable(RCU_SPI2);
  }
#endif
  /* Deinit GPIO mode of SPI pins */
  pin_function(obj->pin_miso, SPI_PINS_FREE_MODE);
  pin_function(obj->pin_mosi, SPI_PINS_FREE_MODE);
  pin_function(obj->pin_sclk, SPI_PINS_FREE_MODE);
  if (handle->params.nss != SPI_NSS_SOFT) {
    pin_function(obj->pin_ssel, SPI_PINS_FREE_MODE);
    spi_nss_output_disable(handle->instance);
  }
}

/**
  * @brief This function is implemented by user to send data over SPI interface (no RX)
  * @param  obj : pointer to spi_t structure
  * @param  data : data to be sent
  * @retval returns the spi status
  */
spi_status_t spi_send(spi_t *obj, uint8_t *data, uint16_t length)
{
  return spi_transfer(obj, data, data, length, 1);
}

/**
  * @brief This function is implemented by user to send/receive data over
  *         SPI interface
  * @param  obj : pointer to spi_t structure
  * @param  tx_buffer : tx data to send before reception
  * @param  rx_buffer : data to receive
  * @param  length : length in byte of the data to send and receive
  * #param  skipRX : skip recieve
  * @retval returns the spi status
  */
spi_status_t spi_transfer(spi_t *obj, uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t length, bool skipRX)
{
  int count = 0;
  spi_status_t spi_status = SPI_OK;
  uint16_t size = length;
  SPIName spiObj = (SPIName)obj->handle.instance;

  if ((obj == NULL) || (length == 0)) {
    return SPI_ERROR;
  }

  while (size--) {
    while ((spi_i2s_flag_get(spiObj, SPI_FLAG_TBE) == RESET) && (count++ < 1000));
    if (count >= 1000) {
      return SPI_TIMEOUT;
    }
    spi_i2s_data_transmit(spiObj, (uint16_t)*tx_buffer++);

    count = 0;
    if (!skipRX) {
      while ((spi_i2s_flag_get(spiObj, SPI_FLAG_TBE) == RESET) && (count++ < 1000));
      if (count >= 1000) {
        return SPI_TIMEOUT;
      }
      *rx_buffer++ = spi_i2s_data_receive(spiObj);
    }
  }

  return spi_status;
}

#ifdef __cplusplus
}
#endif
