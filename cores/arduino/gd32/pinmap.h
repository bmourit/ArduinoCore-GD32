/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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
 */

#ifndef _PINMAP_H
#define _PINMAP_H

#include "PinNames.h"
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint32_t bare_pin_map[16];

#define GD_BARE_GPIO_PIN(X) (bare_pin_map[GD_PIN_GET(X)])

/* provide a way to distiguish between No Peripheral and No Pin */
#define NP		0U

typedef struct {
	PinName pin;
	uint32_t peripheral;
	int function;
} PinMap;

void pin_function(PinName pin, int function);
bool pin_in_pinmap(PinName pin, const PinMap *map);

uint32_t pinmap_peripheral(PinName pin, const PinMap *map);
uint32_t pinmap_function(PinName pin, const PinMap *map);
uint32_t pinmap_merge(uint32_t a, uint32_t b);
void pinmap_pinout(PinName pin, const PinMap *map);
uint32_t pinmap_find_peripheral(PinName pin, const PinMap *map);
uint32_t pinmap_find_function(PinName pin, const PinMap *map);
PinName pinmap_pin(uint32_t peripheral, const PinMap *map);
PinName pinmap_find_pin(uint32_t peripheral, const PinMap *map);

uint32_t gpio_clock_enable(uint32_t port);

#ifdef __cplusplus
}
#endif

#endif /* _PINMAP_H */
