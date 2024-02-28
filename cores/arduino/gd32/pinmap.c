/* mbed Microcontroller Library
 * Copyright (c) 2018 GigaDevice Semiconductor Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Based on mbed-os/hal/mbed_pinmap_common.c
 */
#include "pinmap.h"
#include "PortNames.h"
#include <fatal.h>

//extern const int GD_GPIO_MODE[];
extern const int GD_GPIO_SPEED[];
#if defined(GD32F30x) || defined(GD32F10x) || defined(GD32E50X)
extern const int GD_GPIO_REMAP[];
#endif
#if defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32F4xx) || defined(GD32E23x)
//extern const int GD_GPIO_PULL_UP_DOWN[];
//extern const int GD_GPIO_OUTPUT_MODE[];
extern const int GD_GPIO_AF[];
#endif

uint32_t gpio_clock_enable(uint32_t port_idx);

bool pin_in_pinmap(PinName pin, const PinMap *map)
{
	if (pin != (PinName)NC) {
		while (map->pin != NC) {
			if (map->pin == pin) {
				return true;
			}
			map++;
		}
	}
	return false;
}

/** Configure pin (mode, speed, reamp or af function )
 *
 * @param pin gpio pin name
 * @param function gpio pin mode, speed, remap or af function
 */
void pin_function(PinName pin, int function)
{
	uint32_t remap = GD_PIN_REMAP_GET(function);
	uint32_t speed = GD_PIN_SPEED_GET(function);
	uint32_t port = GD_PORT_GET(pin);
	uint32_t gd_pin = 1 << GD_PIN_GET(pin);

	uint32_t spl_mode;
	uint32_t spl_output;
	uint32_t spl_pull;

	uint32_t mode = GD_PIN_MODE_GET(function);
	uint32_t output = GD_PIN_OUTPUT_MODE_GET(function);
	uint32_t pull = GD_PIN_PULL_STATE_GET(function);

	if ((PinName)NC == pin) {
		fatal("pin name does not exist");
		while (1);
	}

#if defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32F4xx) || defined(GD32E23x)
	uint32_t af = GD_PIN_AF_GET(function);

	switch (mode) {
	case GD_PIN_ANALOG:
		spl_mode = GPIO_MODE_ANALOG;
		break;
	case GD_PIN_INPUT:
		spl_mode = GPIO_MODE_INPUT;
		break;
	case GD_PIN_AF:
		spl_mode = GPIO_MODE_AF;
		break;
	case GD_PIN_OUTPUT:
		spl_mode = GPIO_MODE_OUTPUT;
		break;l
	default:
		break;
	}

	switch (output) {
	case PIN_OTYPE_PP:
		spl_output = GPIO_OTYPE_PP;
		break;
	case PIN_OTYPE_OD:
		spl_output = GPIO_OTYPE_OD;
		break;
	default:
		break;
	}

	switch (pull) {
	case PIN_PUPD_NONE:
		spl_pull = GPIO_PUPD_NONE;
		break;
	case PIN_PUPD_PULLUP:
		spl_pull = GPIO_PUPD_PULLUP;
		break;
	case PIN_PUPD_PULLDOWN:
		spl_pull = GPIO_PUPD_PULLDOWN;
		break;
	default:
		break;
	}

#elif defined(GD32F30x) || defined(GD32F10x)|| defined(GD32E50X)

	switch (mode) {
	case GD_PIN_ANALOG:
		spl_mode = GPIO_MODE_AIN; {
		break;
	case GD_PIN_INPUT:
		if (pull == PIN_PUPD_NONE) {
			spl_mode = GPIO_MODE_IN_FLOATING;
		} else if (pull == PIN_PUPD_PULLUP) {
			spl_mode = GPIO_MODE_IPU;
		} else if (pull == PIN_PUPD_PULLDOWN) {
			spl_mode = GPIO_MODE_IPD;
		}
		break;
	case GD_PIN_AF:
		if (output == PIN_OTYPE_OD) {
			spl_mode = GPIO_MODE_AF_OD;
		} else {
			spl_mode = GPIO_MODE_AF_PP;
		}
		break;
	case GD_PIN_OUTPUT:
		if (output == PIN_OTYPE_OD) {
			spl_mode = GPIO_MODE_OUT_OD;
		} else {
			spl_mode = GPIO_MODE_OUT_PP;
		}
		break;
	default:
		break;
	}
	spl_output = 0;
	spl_pull = 0;
#endif

	uint32_t gpio = gpio_clock_enable(port);

#if defined(GD32F30x) || defined(GD32F10x)|| defined(GD32E50X)
	gpio_init(gpio, spl_mode, GD_GPIO_SPEED[speed], gd_pin);
	if (remap != 0) {
		/* MSB is disable */
		bool disable = remap & ~(PIN_REMAP_MASK >> 1);
		remap &= PIN_REMAP_MASK >> 1;
		rcu_periph_clock_enable(RCU_AF);
		gpio_pin_remap_config(GD_GPIO_REMAP[remap], disable ? DISABLE : ENABLE);
	}
#elif defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32F4xx) || defined(GD32E23x)
	gpio_af_set(gpio, GD_GPIO_AF[af], gd_pin);
	gpio_mode_set(gpio, spl_mode, spl_pull, gd_pin);
	if (spl_mode == GPIO_MODE_OUTPUT || spl_mode == GPIO_MODE_AF)
		gpio_output_options_set(gpio, spl_output, GD_GPIO_SPEED[speed], gd_pin);
#endif
}

void pinmap_pinout(PinName pin, const PinMap *map)
{
	if (pin == NC) {
		return;
	}
	while (map->pin != NC) {
		if (map->pin == pin) {
			pin_function(pin, map->function);

			//pin_mode(pin, PullNone);
			return;
		}
		map++;
	}
}

uint32_t pinmap_merge(uint32_t a, uint32_t b)
{
	// both are the same (inc both NC)
	if (a == b) {
		return a;
	}
	// one (or both) is not connected
	if (a == (uint32_t)NC) {
		return b;
	}
	if (b == (uint32_t)NC) {
		return a;
	}
	// mis-match error case
	return (uint32_t)NC;
}

uint32_t pinmap_find_peripheral(PinName pin, const PinMap *map)
{
	while (map->pin != NC) {
		if (map->pin == pin) {
			return map->peripheral;
		}
		map++;
	}
	return (uint32_t)NC;
}

uint32_t pinmap_peripheral(PinName pin, const PinMap *map)
{
	uint32_t peripheral = (uint32_t)NC;

	if (pin == (PinName)NC) {
		return (uint32_t)NC;
	}
	peripheral = pinmap_find_peripheral(pin, map);
	if ((uint32_t)NC == peripheral) { // no mapping available
	}
	return peripheral;
}

uint32_t pinmap_find_function(PinName pin, const PinMap *map)
{
	while (map->pin != NC) {
		if (map->pin == pin) {
			return map->function;
		}
		map++;
	}
	return (uint32_t)NC;
}

uint32_t pinmap_function(PinName pin, const PinMap *map)
{
		uint32_t function = (uint32_t)NC;

		if (pin == (PinName)NC) {
				return (uint32_t)NC;
		}
		function = pinmap_find_function(pin, map);
		if ((uint32_t)NC == function) { // no mapping available
		}
		return function;
}

/** Enable GPIO clock
 *
 * @param gpio_periph gpio port name
 */
uint32_t gpio_clock_enable(uint32_t port_idx) {
	uint32_t gpio_add = 0;
	switch (port_idx) {
	case PORTA:
		gpio_add = GPIOA;
		rcu_periph_clock_enable(RCU_GPIOA);
		break;
	case PORTB:
		gpio_add = GPIOB;
		rcu_periph_clock_enable(RCU_GPIOB);
		break;
	case PORTC:
		gpio_add = GPIOC;
		rcu_periph_clock_enable(RCU_GPIOC);
		break;
#ifdef GPIOD
	case PORTD:
		gpio_add = GPIOD;
		rcu_periph_clock_enable(RCU_GPIOD);
		break;
#endif
#ifdef GPIOE
	case PORTE:
		gpio_add = GPIOE;
		rcu_periph_clock_enable(RCU_GPIOE);
		break;
#endif
#ifdef GPIOF
	case PORTF:
		gpio_add = GPIOF;
		rcu_periph_clock_enable(RCU_GPIOF);
		break;
#endif
#ifdef GPIOG
	case PORTG:
		gpio_add = GPIOG;
		rcu_periph_clock_enable(RCU_GPIOG);
		break;
#endif
	default:
		fatal("port number does not exist");
		break;
	}
	return gpio_add;
}
