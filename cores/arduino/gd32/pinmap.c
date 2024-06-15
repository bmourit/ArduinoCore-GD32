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
#include "gd_debug.h"
#include "gd32f30x_remap.h"

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
    if (pin == NC) {
        return false;
    }

    uint32_t idx = pin - map->pin;
    const PinMap *end = map + (sizeof(*map) / sizeof(map->pin));

    while (map < end) {
        if (map->pin == pin) {
            return true;
        }
        if (idx < idx - map->pin) {
            return false;
        }
        map++;
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
	uint8_t mode = GD_PIN_MODE_GET(function);
	uint8_t af = GD_PIN_AF_GET(function);
	uint8_t port = GD_PORT_GET(pin);
	uint32_t spl_pin = APIN_TO_GPIN(GD_PIN_GET(pin));
	uint32_t spl_mode = 0;
	uint8_t output = GD_PIN_OUTPUT_OD_GET(function);
	uint8_t pull = GD_PIN_PULL_UD_GET(function);

	if (pin == (PinName)NC) {
		Error_Handler();
	}

	uint32_t gpio = gpio_clock_enable(port);

#if defined(GD32F3x0) || defined(GD32F1x0) || defined(GD32F4xx) || defined(GD32E23x)
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
	}

	gpio_af_set(gpio, GD_GPIO_AF[af], spl_pin);
	gpio_mode_set(gpio, spl_mode, pull, spl_pin);
	if (spl_mode == GPIO_MODE_OUTPUT || spl_mode == GPIO_MODE_AF)
		gpio_output_options_set(gpio, output, GPIO_OSPEED_MAX, spl_pin);
#else
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
			spl_mode = output == PIN_OTYPE_OD ? GPIO_MODE_AF_OD : GPIO_MODE_AF_PP;
			f3_afpin_set(af);
			break;
		case GD_PIN_OUTPUT:
			spl_mode = output == PIN_OTYPE_OD ? GPIO_MODE_OUT_OD : GPIO_MODE_OUT_PP;
			break;
	}
	gpio_init(gpio, spl_mode, GPIO_OSPEED_MAX, spl_pin);
	gpio_compensation_config(GPIO_COMPENSATION_ENABLE);
	f3_debug_disconnect(pin);
#endif
}

void pinmap_pinout(PinName pin, const PinMap *map)
{
	const PinMap *end = map + (sizeof(*map) / sizeof(map->pin));
	const PinMap *l = map;

	while (l < end) {
		if (l->pin == pin) {
			pin_function(pin, l->function);
			return;
		}
		l++;
	}
}

uint32_t pinmap_merge(uint32_t a, uint32_t b)
{
	// return the non-NC value if both are the same
	// or if one is not connected
	return (a == b || a == NP) ? a :
		(b == NP) ? a :
		(a == NP) ? b :
		// mis-match error case
		NP;
}

uint32_t pinmap_find_peripheral(PinName pin, const PinMap *map)
{
	const PinMap *end = map + (sizeof(*map) / sizeof(map->pin));
	const PinMap *l = map;

	while (l < end) {
		if (l->pin == pin) {
			return l->peripheral;
		}
		l++;
	}
	return NP;
}

uint32_t pinmap_peripheral(PinName pin, const PinMap *map)
{
	const PinMap *end = map + (sizeof(*map) / sizeof(map->pin));
	const PinMap *l = map;

	while (pin != NC && l < end && l->pin != pin) {
		l++;
	}
	return (pin != NC && l < end) ? l->peripheral : NP;
}

uint32_t pinmap_find_function(PinName pin, const PinMap *map)
{
	const PinMap *end = map + (sizeof(*map) / sizeof(map->pin));
	const PinMap *l = map;

	while (l < end && l->pin != pin) {
		l++;
	}
	return (l < end) ? l->function : NC;
}

uint32_t pinmap_function(PinName pin, const PinMap *map)
{
    while (pin != NC && map->pin != pin) {
        map++;
    }
    return (pin != NC && map->pin == pin) ? map->function : NC;
}

PinName pinmap_find_pin(uint32_t peripheral, const PinMap *map)
{
    const PinMap *end = map + (sizeof(*map) / sizeof(map->peripheral));
    const PinMap *p = map;

    while (p < end && p->peripheral != peripheral)
        ++p;

    return (p < end) ? p->pin : NC;
}

PinName pinmap_pin(uint32_t peripheral, const PinMap *map)
{
    const PinMap *end = map + (sizeof(*map) / sizeof(map->peripheral));
    const PinMap *p = map;

    while (p < end && p->peripheral != peripheral)
        ++p;

    return (p < end) ? p->pin : NC;
}

/**
 * Enable GPIO clock
 * @param gpio_periph gpio port name
 */
uint32_t gpio_clock_enable(uint32_t port)
{
	uint32_t gpiox = NP;
	static const uint32_t clocks[] = {
		[PORTA] = RCU_GPIOA,
		[PORTB] = RCU_GPIOB,
		[PORTC] = RCU_GPIOC,
		[PORTD] = RCU_GPIOD,
	};

	if (port < sizeof(clocks)/sizeof(clocks[0])) {
		rcu_periph_clock_enable(clocks[port]);
		gpiox = port;
	} else {
		gd_debug("port number does not exist");
	}

	return gpiox;
}
