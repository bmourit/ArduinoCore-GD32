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
#include "pins_arduino.h"
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint32_t bare_pin_map[16];

#define GD_BARE_GPIO_PIN(X) (bare_pin_map[GD_PIN_GET(X)])

enum GD_GPIO_REMAP_NAME {
  REMAP_NONE = 0U,
#if __has_include("gd32f30x_remap.h")
#define __REMAP_NAME__(remap) remap,
#include "gd32f30x_remap.h"
#undef __REMAP_NAME__
#endif
};

/* provide a way to distiguish between No Peripheral and No Pin */
#define NP      0U

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

void gpio_clock_enable(uint32_t gpio_port);

static inline void gpio_debug_disconnect(PinName pin)
{
#if defined(GD32F30x) || defined(GD32F10x)
  /* Enable this flag gives the possibility to use debug pins without any risk to lose traces */
#ifndef LOCK_LOWLEVEL_DEBUG
  rcu_periph_clock_enable(RCU_AF);

  /* JTAG-DP disabled and SW-DP disabled */
  if ((pin == PORTA_13) || (pin == PORTA_14)) {
    gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_SWJ_DISABLE_REMAP, ENABLE);      
  }
  /* JTAG-DP disabled and SW-DP enabled */
  if ((pin == PORTA_15) || (pin == PORTB_3) || (pin == PORTB_4)) {
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
  }
#else
    UNUSED(pin);
#endif
#else
  UNUSED(pin);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* _PINMAP_H */
