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
        stop_pwm(p);
      }
      RESET_PIN_CONFIG(p, gpioOutputPinIsConfigured);
    }
    switch (ulMode) {
      case INPUT:
        pin_function(p, GD_PIN_DATA(PIN_MODE_INPUT, PIN_PUPD_NONE, 0));
        break;
      case INPUT_PULLUP:
        pin_function(p, GD_PIN_DATA(PIN_MODE_INPUT, PIN_PUPD_PULLUP, 0));
        break;
      case INPUT_PULLDOWN:
        pin_function(p, GD_PIN_DATA(PIN_MODE_INPUT, PIN_PUPD_PULLDOWN, 0));
        break;
      case OUTPUT:
        pin_function(p, GD_PIN_DATA(PIN_MODE_OUT_PP, PIN_PUPD_NONE, 0));
        break;
#pragma GCC diagnostic ignored "-Wswitch"
      case INPUT_ANALOG: // From PinModeExtension
        pin_function(p, GD_PIN_DATA(PIN_MODE_ANALOG, PIN_PUPD_NONE, 0));
      break;
#pragma GCC diagnostic ignored "-Wswitch"
      case OUTPUT_OPEN_DRAIN: // From PinModeExtension
        pin_function(p, GD_PIN_DATA(PIN_MODE_OUT_OD, PIN_PUPD_NONE, 0));
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
  uint32_t gpiopin =  gpio_pin[GD_PIN_GET(pinname)];
  uint32_t port = gpio_port[GD_PORT_GET(pinname)];
  gpio_bit_write(port, gpiopin, (bit_status)status);
}

PinStatus digitalRead(pin_size_t pin)
{
  PinName pinname = DIGITAL_TO_PINNAME(pin);
  uint32_t gpiopin =  gpio_pin[GD_PIN_GET(pinname)];
  uint32_t port = gpio_port[GD_PORT_GET(pinname)];
  return (FlagStatus)gpio_input_bit_get(port, gpiopin);
}

void digitalToggle(pin_size_t pin)
{
  PinName pinname = DIGITAL_TO_PINNAME(pin);
  uint32_t gpiopin =  gpio_pin[GD_PIN_GET(pinname)];
  uint32_t port = gpio_port[GD_PORT_GET(pinname)];
  gpio_bit_write(port, gpiopin, (bit_status)(1 - (FlagStatus)gpio_input_bit_get(port, gpiopin)));
}

/**
 * converts PORTx to GPIOx that correspond to the register base addresses
 * 
 * Problem: These are defined per mcu family in the header file, which means
 * they could be defined even if the specific mcu chip doesn't have them
 * 
 * TODO: consider adding a file for undefining these beforehand on a per-chip basis.
 */
const uint32_t gpio_port[] = {
  GPIOA,
  GPIOB,
  GPIOC,
#ifdef GPIOD
  GPIOD,
#endif
#ifdef GPIOE
  GPIOE,
#endif
#ifdef GPIOF
  GPIOF,
#endif
#ifdef GPIOG
  GPIOG,
#endif
#ifdef GPIOH
  GPIOH,
#endif
#ifdef GPIOI
  GPIOI
#endif
};

/* converts pinname to gpio pin bit location in register */
const uint32_t gpio_pin[] = {
  GPIO_PIN_0,
  GPIO_PIN_1,
  GPIO_PIN_2,
  GPIO_PIN_3,
  GPIO_PIN_4,
  GPIO_PIN_5,
  GPIO_PIN_6,
  GPIO_PIN_7,
  GPIO_PIN_8,
  GPIO_PIN_9,
  GPIO_PIN_10,
  GPIO_PIN_11,
  GPIO_PIN_12,
  GPIO_PIN_13,
  GPIO_PIN_14,
  GPIO_PIN_15
};

#ifdef __cplusplus
}
#endif

