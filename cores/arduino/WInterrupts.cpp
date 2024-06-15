/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

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
#include "gd32f30x_remap.h"
#include "gpio_interrupt.h"

void attachInterrupt(pin_size_t pin, voidFuncPtr callback, PinStatus mode)
{
#if defined(SPL_EXTI_ENABLE)
    exti_trig_type_enum it_mode;
    PinName p = DIGITAL_TO_PINNAME(pin);
    uint32_t gpioPort = set_gpio_port_clock(GD_PORT_GET(p));

    if (!gpioPort) {
        return;
    }

    switch (mode) {
        case CHANGE:
            it_mode = EXTI_TRIG_BOTH;
            break;
        case FALLING:
        case LOW:
            it_mode = EXTI_TRIG_FALLING;
            break;
        case RISING:
        case HIGH:
            it_mode = EXTI_TRIG_RISING;
            break;
        default:
            it_mode = EXTI_TRIG_RISING;
            break;
    }
#ifdef GD32F30x
    f3_debug_disconnect(p);
#endif
    gpio_interrupt_enable(gpioPort, GD_GPIO_PIN(p), callback, it_mode);
#else
    UNUSED(pin);
    UNUSED(callback);
    UNUSED(mode);
#endif
}

void detachInterrupt(pin_size_t pin)
{
#if defined(SPL_EXTI_ENABLE)
    PinName p = DIGITAL_TO_PINNAME(pin);
    uint32_t gpioPort = GET_GPIO_PORT(GD_PORT_GET(p));
    if (!gpioPort) {
        return;
    }
    gpio_interrupt_disable(gpioPort, GD_GPIO_PIN(p));
#else
    UNUSED(pin);
#endif
}
