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

#include "PortNames.h"

uint32_t set_gpio_port_clock(uint32_t port)
{
	uint32_t gpioPort = 0;

	switch (port) {
		case PORTA:
			gpioPort = GPIOA;
			rcu_periph_clock_enable(RCU_GPIOA);
			break;
		case PORTB:
			gpioPort = GPIOB;
			rcu_periph_clock_enable(RCU_GPIOB);
			break;
		case PORTC:
			gpioPort = GPIOC;
			rcu_periph_clock_enable(RCU_GPIOC);
			break;
#if defined(GPIOD)
		case PORTD:
			gpioPort = GPIOD;
			rcu_periph_clock_enable(RCU_GPIOD);
			break;
#endif
#if defined(GPIOE)
		case PORTE:
			gpioPort = GPIOE;
			rcu_periph_clock_enable(RCU_GPIOE);
			break;
#endif
		default:
			gpioPort = 0;
			break;
	}

	return gpioPort;
}
