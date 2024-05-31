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
#include "gd32f30x_remap.h"
#include <gd_debug.h>
#include <gd32xxyy_gpio.h>

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

/**
 * Configure pin (mode, speed, output type and pull-up/pull-down)
 * @param pin pin name
 * @param function bits to decode
 */
void pin_function(PinName pin, int function)
{
	uint32_t mode = GD_PIN_MODE_GET(function);
	uint32_t af = GD_PIN_AF_GET(function);
	uint32_t port = GD_PORT_GET(pin);
	uint32_t spl_pin = gpio_pin[GD_PIN_GET(pin)];
	uint8_t spl_mode = 0;
	uint32_t output = GD_PIN_OUTPUT_OD_GET(function);
	uint32_t pull = GD_PIN_PULL_UD_GET(function);
	/**
	 * since we dont encode the speed in the function,
	 * we can simply set it to MAX here
	 */
	uint8_t speed = GPIO_OSPEED_MAX;

	if (pin == (PinName)NC) {
		Error_Handler();
	}

#if defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32F4xx) || defined(GD32E23x)
	uint8_t spl_output = 0;
	uint8_t spl_pull = 0;

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
		break;
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
#endif

#if defined(GD32F30x) || defined(GD32F10x) || defined(GD32E50X)

	switch (mode) {
	case GD_PIN_ANALOG:
		spl_mode = GPIO_MODE_AIN;
		break;
	case GD_PIN_INPUT:
		if (pull == PIN_PUPD_PULLUP) {
			spl_mode = GPIO_MODE_IPU;
		} else if (pull == PIN_PUPD_PULLDOWN) {
			spl_mode = GPIO_MODE_IPD;
		} else {
			spl_mode = GPIO_MODE_IN_FLOATING;
		}
		f3_afpin_set(af);
		break;
	case GD_PIN_AF:
		if (output == PIN_OTYPE_OD) {
			spl_mode = GPIO_MODE_AF_OD;
		} else {
			spl_mode = GPIO_MODE_AF_PP;
		}
		f3_afpin_set(af);
		break;
	case GD_PIN_OUTPUT:
		if (output == PIN_OTYPE_OD) {
			spl_mode = GPIO_MODE_OUT_OD;
		} else {
			spl_mode = GPIO_MODE_OUT_PP;
		}
		break;
	default:
		Error_Handler();
		break;
	}
#endif

	uint32_t gpio = gpio_clock_enable(port);

#if defined(GD32F30x) || defined(GD32F10x) || defined(GD32E50X)
	gpio_init(gpio, spl_mode, speed, spl_pin);
	gpio_compensation_config(GPIO_COMPENSATION_ENABLE);
	f3_debug_disconnect(pin);
#endif
#if defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32F4xx) || defined(GD32E23x)
	gpio_af_set(gpio, GD_GPIO_AF[af], spl_pin);
	gpio_mode_set(gpio, spl_mode, spl_pull, spl_pin);
	//gpio_debug_disconnect(pin);
	if (spl_mode == GPIO_MODE_OUTPUT || spl_mode == GPIO_MODE_AF)
		gpio_output_options_set(gpio, spl_output, speed, spl_pin);
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
	if (a == NP) {
		return b;
	}
	if (b == NP) {
		return a;
	}
	// mis-match error case
	return NP;
}

uint32_t pinmap_find_peripheral(PinName pin, const PinMap *map)
{
	while (map->pin != NC) {
		if (map->pin == pin) {
			return map->peripheral;
		}
		map++;
	}
	return NP;
}

uint32_t pinmap_peripheral(PinName pin, const PinMap *map)
{
	uint32_t peripheral = NP;

	if (pin != (PinName)NC) {
		peripheral = pinmap_find_peripheral(pin, map);
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
	return NC;
}

uint32_t pinmap_function(PinName pin, const PinMap *map)
{
		uint32_t function = NC;

		if (pin != (PinName)NC) {
			function = pinmap_find_function(pin, map);
		}
		return function;
}

PinName pinmap_find_pin(uint32_t peripheral, const PinMap *map)
{
  while (map->peripheral != NP) {
    if (map->peripheral == peripheral) {
      return map->pin;
    }
    map++;
  }
  return NC;
}

PinName pinmap_pin(uint32_t peripheral, const PinMap *map)
{
  PinName pin = (PinName)NC;

  if (peripheral != NP) {
    pin = pinmap_find_pin(peripheral, map);
  }
  return pin;
}

/**
 * Enable GPIO clock
 * @param gpio_periph gpio port name
 */
uint32_t gpio_clock_enable(uint32_t port)
{
	uint32_t gpiox = NP;
	switch (port) {
	case PORTA:
		rcu_periph_clock_enable(RCU_GPIOA);
		gpiox = GPIOA;
		break;
	case PORTB:
		rcu_periph_clock_enable(RCU_GPIOB);
		gpiox = GPIOB;
		break;
	case PORTC:
		rcu_periph_clock_enable(RCU_GPIOC);
		gpiox = GPIOC;
		break;
#ifdef GPIOD
	case PORTD:
		rcu_periph_clock_enable(RCU_GPIOD);
		gpiox = GPIOD;
		break;
#endif
#ifdef GPIOE
	case PORTE:
		rcu_periph_clock_enable(RCU_GPIOE);
		gpiox = GPIOE;
		break;
#endif
#ifdef GPIOF
	case PORTF:
		rcu_periph_clock_enable(RCU_GPIOF);
		gpiox = GPIOF;
		break;
#endif
#ifdef GPIOG
	case PORTG:
		rcu_periph_clock_enable(RCU_GPIOG);
		gpiox = GPIOG;
		break;
#endif
	default:
		gd_debug("port number does not exist");
		break;
	}
	return gpiox;
}
