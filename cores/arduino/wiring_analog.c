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

#include "PinConfigured.h"
#include "gd32/PinNames.h"
#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PWM_RESOLUTION  16
#define MAX_ADC_RESOLUTION  12

static uint32_t analogIn_resolution = ADC_RESOLUTION;
static uint32_t analogOut_resolution = PWM_RESOLUTION;
static uint32_t internalAnalogOut_resolution = (PWM_RESOLUTION > MAX_PWM_RESOLUTION) ? MAX_PWM_RESOLUTION : PWM_RESOLUTION;
static uint32_t analogOut_period_us = PWM_FREQUENCY;

void analogReference(uint8_t mode)
{
  // TODO: implement this
  (void) mode;
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
int analogRead(pin_size_t ulPin)
{
  uint32_t value = 0;
  PinName p = DIGITAL_TO_PINNAME(ulPin);

  /* set pin mode to analog input before read */
  pinMode(ulPin, INPUT_ANALOG);
  switch (ulPin) {
    case ATEMP:
      p = ADC_TEMP;
      break;
    case AVREF:
      p = ADC_VREF;
      break;
    default:
      break;
  }
  if (p != NC) {
    value = get_adc_value(p);
    value = mapResolution(value, MAX_ADC_RESOLUTION, analogIn_resolution);
  }

  return value;
}

// Right now, PWM output only works on the pins with
// hardware support. These are defined in the appropriate
// variant.cpp file. For the rest of the pins, we default
// to digital output.
void analogWrite(pin_size_t pin, int value)
{
  uint32_t ulValue;
  PinName pinname = DIGITAL_TO_PINNAME(pin);
  if (pin_in_pinmap(pinname, PinMap_DAC)) {
    ulValue = mapResolution(value, analogOut_resolution, DAC_RESOLUTION);
    set_dac_value(pinname, ulValue);
  } else if (pin_in_pinmap(pinname, PinMap_PWM)) {
    ulValue = mapResolution(value, analogOut_resolution, internalAnalogOut_resolution);
    /* handle special cases: 100% off and 100% on */
    if (ulValue == 0) {
      stop_pwm(pin);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    } else if (ulValue == UINT16_MAX) {
      stop_pwm(pin);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
    } else {
      set_pwm_value_with_base_period(pin, analogOut_period_us, ulValue);
    }
  } else {
    /* default to digital write */
    pinMode(pin, OUTPUT);
    ulValue = mapResolution(value, analogOut_resolution, 8);
    if (ulValue < 128) {
      digitalWrite(pin, LOW);
    } else {
      digitalWrite(pin, HIGH);
    }
  }
}

/* analog input resolution */
void analogReadResolution(int res)
{
  if ((res > 0) && (res < 16)) {
    analogIn_resolution = res;
  } else {
    analogIn_resolution = 8;
  }
}

/* analog output resolution */
void analogWriteResolution(int res)
{
  if ((res > 0) && (res <= 16)) {
    if (res > MAX_PWM_RESOLUTION) {
      res = MAX_PWM_RESOLUTION;
    }
    analogOut_resolution = res;
  } else {
    analogOut_resolution = 8;
  }
}

void analogWriteFrequency(uint32_t freq_hz) {
  analogOut_period_us = 1000000U / freq_hz;
}

#ifdef __cplusplus
}
#endif
