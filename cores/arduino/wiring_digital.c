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

extern uint32_t gpioOutputPinIsConfigured[GPIO_PORT_NUM];

void pinMode(pin_size_t pin, PinMode ulMode)
{
  PinName p = DIGITAL_TO_PINNAME(pin);

  if (p != NC) {
    if (PIN_IS_CONFIG(p, gpioOutputPinIsConfigured)) {
      if (pin_in_pinmap(p, PinMap_DAC)) {
        dac_stop(p);
      } else if (pin_in_pinmap(p, PinMap_PWM)) {
        pwm_stop(p);
      }
      RESET_PIN_CONFIG(p, gpioOutputPinIsConfigured);
    }
    switch (ulMode) {
      case INPUT:
        pin_function(p, GD_PIN_DATA(GD_MODE_INPUT, PIN_PUPD_NONE, 0));
        break;
      case INPUT_PULLUP:
        pin_function(p, GD_PIN_DATA(GD_MODE_INPUT, PIN_PUPD_PULLUP, 0));
        break;
      case INPUT_PULLDOWN:
        pin_function(p, GD_PIN_DATA(GD_MODE_INPUT, PIN_PUPD_PULLDOWN, 0));
        break;
      case OUTPUT:
        pin_function(p, GD_PIN_DATA(GD_MODE_OUT_PP, PIN_PUPD_NONE, 0));
        break;
//#pragma GCC diagnostic ignored "-Wswitch"
      case INPUT_ANALOG: // From PinModeExtension
        pin_function(p, GD_PIN_DATA(GD_MODE_ANALOG, PIN_PUPD_NONE, 0));
      break;
//#pragma GCC diagnostic ignored "-Wswitch"
      case OUTPUT_OPEN_DRAIN: // From PinModeExtension
        pin_function(p, GD_PIN_DATA(GD_MODE_OUT_OD, PIN_PUPD_NONE, 0));
        break;
      default:
        Error_Handler();
        break;
    }
  }
}

void digitalWrite(pin_size_t pin, PinStatus status)
{
  PinName pinname = DIGITAL_TO_PINNAME(pin);
  uint32_t gpiopin =  APIN_TO_GPIN(GD_PIN_GET(pinname));
  uint32_t port = APORT_TO_GPORT(GD_PORT_GET(pinname));
  gpio_bit_write(port, gpiopin, (bit_status)status);
}

PinStatus digitalRead(pin_size_t pin)
{
  PinName pinname = DIGITAL_TO_PINNAME(pin);
  uint32_t gpiopin =  APIN_TO_GPIN(GD_PIN_GET(pinname));
  uint32_t port = APORT_TO_GPORT(GD_PORT_GET(pinname));
  return (FlagStatus)gpio_input_bit_get(port, gpiopin);
}

void digitalToggle(pin_size_t pin)
{
  PinName pinname = DIGITAL_TO_PINNAME(pin);
  uint32_t gpiopin =  APIN_TO_GPIN(GD_PIN_GET(pinname));
  uint32_t port = APORT_TO_GPORT(GD_PORT_GET(pinname));
  gpio_bit_write(port, gpiopin, (bit_status)(1 - (FlagStatus)gpio_input_bit_get(port, gpiopin)));
}

#ifdef __cplusplus
}
#endif
