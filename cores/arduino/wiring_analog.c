/*
 Copyright (c) 2011 Arduino.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "Arduino.h"
#include "PinConfigured.h"

#ifdef __cplusplus
extern "C" {
#endif

/* adds functionality by providing a list of configured IOs */
uint32_t gpioOutputPinIsConfigured[GPIO_PORT_NUM] = { 0 };

#define MAX_PWM_RESOLUTION  16
#define MAX_ADC_RESOLUTION  12

static int _analog_read_resolution = ADC_RESOLUTION;
static int internal_analog_read_resolution = (ADC_RESOLUTION > MAX_ADC_RESOLUTION) ? MAX_ADC_RESOLUTION : ADC_RESOLUTION;

static int _analog_write_resolution = PWM_RESOLUTION;
static int internal_analog_write_resolution = (PWM_RESOLUTION > MAX_PWM_RESOLUTION) ? MAX_PWM_RESOLUTION : PWM_RESOLUTION;

static uint32_t _analog_write_freq = PWM_FREQUENCY;


void analogReference(uint8_t mode)
{
  // TODO: implement this
  //(void)mode;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from != to) {
    if (from > to) {
      value = (value < (uint32_t)(1 << (from - to))) ? 0 : ((value + 1) >> (from - to)) - 1;
    } else {
      if (value != 0) {
        value = ((value + 1) << (to - from)) - 1;
      }
    }
  }
  return value;
}

// Perform the read operation on the selected analog pin.
// the initialization of the analog PIN is done through this function
int analogRead(pin_size_t pin)
{
  uint32_t value = 0;
  PinName p = analog_pin_to_PinName(pin);
  if (p != NC) {
    value = get_adc_value(p, internal_analog_read_resolution);
    value = mapResolution(value, internal_analog_read_resolution, _analog_read_resolution);
  }

  return value;
}

// Right now, PWM output only works on the pins with
// hardware support. These are defined in the appropriate
// variant.cpp file. For the rest of the pins, we default
// to digital output.
void analogWrite(pin_size_t pin, int value)
{
  uint8_t needs_init = 0;
  //uint32_t ulValue;
  PinName pn = DIGITAL_TO_PINNAME(pin);
  if (pn != NC) {
    if (pin_in_pinmap(pn, PinMap_DAC)) {
      if (PIN_IS_CONFIG(pn, gpioOutputPinIsConfigured) == false) {
        needs_init = 1;
        SET_PIN_CONFIG(pn, gpioOutputPinIsConfigured);
      }
      value = mapResolution(value, _analog_write_resolution, DAC_RESOLUTION);
      set_dac_value(pn, value, needs_init);
    } else if (pin_in_pinmap(pn, PinMap_PWM)) {
      if (PIN_IS_CONFIG(pn, gpioOutputPinIsConfigured) == false) {
        SET_PIN_CONFIG(pn, gpioOutputPinIsConfigured);
      }
      value = mapResolution(value, _analog_write_resolution, internal_analog_write_resolution);
      /* handle special cases: 100% off and 100% on */
      pwm_start(pn, _analog_write_freq, value, internal_analog_write_resolution);
    } else {
      /* default to digital write */
      pinMode(pin, OUTPUT);
      value = mapResolution(value, _analog_write_resolution, 8);
      if (value < 128) {
        digitalWrite(pin, LOW);
      } else {
        digitalWrite(pin, HIGH);
      }
    }
  }
}

/* analog input resolution */
void analogReadResolution(int res)
{
  if ((res > 0) && (res <= 12)) {
    _analog_read_resolution = res;
    internal_analog_read_resolution = _analog_read_resolution;
    if (_analog_read_resolution > MAX_ADC_RESOLUTION) {
      internal_analog_read_resolution = MAX_ADC_RESOLUTION;
    } else {
#ifdef ADC_RESOLUTION_6B
      if (internal_analog_read_resolution <= 6) {
        internal_analog_read_resolution = 6;
      }
#endif
      if (internal_analog_read_resolution <= 8) {
        internal_analog_read_resolution = 8;
      } else if (internal_analog_read_resolution <= 10) {
        internal_analog_read_resolution = 10;
      } else if (internal_analog_read_resolution <= 12) {
        internal_analog_read_resolution = 12;
      } else
        internal_analog_read_resolution = 8;
    }
  }
}

/* analog output resolution */
void analogWriteResolution(int res)
{
  if ((res > 0) && (res <= 16)) {
    _analog_write_resolution = res;
    if (_analog_write_resolution > MAX_PWM_RESOLUTION) {
      internal_analog_write_resolution = MAX_PWM_RESOLUTION;
    } else {
      internal_analog_write_resolution = _analog_write_resolution;
    }
  } else {
    Error_Handler();
  }
}

void analogWriteFrequency(uint32_t freq) {
  _analog_write_freq = freq;
}

#ifdef __cplusplus
}
#endif
