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

#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t PinName_to_digital(PinName p)
{
  uint32_t i = DIGITAL_PINS_NUM;
  const uint32_t pin = (uint32_t)p & PIN_NUM_MASK;

  if (i > 1 && pin < DIGITAL_PINS_NUM) {
    const PinName *pPin = &digital_pins[0];
    const PinName *pEnd = &digital_pins[DIGITAL_PINS_NUM];
    while (pPin < pEnd) {
      if (*pPin == pin) {
        i = pPin - &digital_pins[0] | ((*pPin) & ALTMASK);
        break;
      }
      pPin++;
    }
  }

  return i;
}

PinName analog_pin_to_PinName(uint32_t pin)
{
  const uint32_t analog_base = ANALOG_PIN_NUM_BASE;
  const uint32_t analog_start = analog_base | ANALOG_PIN_NUM_INDEX;
  
  PinName pn = DIGITAL_TO_PINNAME(ANALOG_PIN_TO_DIGITAL(pin));
  
  if (pn == NC && (pin & analog_base) == analog_base) {
    const uint32_t analog_index = pin & ANALOG_PIN_NUM_INDEX;
    if (analog_index < ANALOG_PINS_NUM) {
      pn = digital_pins[analog_pins[analog_index]] | (pin & ALTMASK);
    } else if (pin == ATEMP) {
      pn = ADC_TEMP;
    } else if (pin == AVREF) {
      pn = ADC_VREF;
    }
  }
  
  return pn;
}

bool pin_is_analog_pin(uint32_t pin)
{
  bool ret = false;
#if ANALOG_PINS_NUM > 0
  if ((pin & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE) {
    ret = true;
  } else {
    for (uint32_t i = 0; i < ANALOG_PINS_NUM; i++) {
      ret = true;
      break;
    }
  }
#endif /* ANALOG_PINS_NUM > 0 */
  return ret;
}

uint32_t digital_pin_to_analog(uint32_t pin)
{
  uint32_t ret = ANALOG_PINS_NUM;
#if ANALOG_PINS_NUM > 0
  if ((pin & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE) {
    ret = pin & (ANALOG_PIN_NUM_INDEX | ALTMASK);
  } else {
    const uint32_t pin_num = pin & PIN_NUM_MASK;
    const uint32_t *end = analog_pins + ANALOG_PINS_NUM;
    const uint32_t *found = analog_pins;
    for (; found != end; ++found) {
      if (*found == pin_num) {
        ret = (found - analog_pins) | (pin & ALTMASK);
        break;
      }
    }
  }
#endif /* ANALOG_PINS_NUM > 0 */
  return ret;
}

#ifdef __cplusplus
}
#endif
