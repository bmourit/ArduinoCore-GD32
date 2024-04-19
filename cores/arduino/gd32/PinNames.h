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
 */
#ifndef _PINNAMES_H
#define _PINNAMES_H

#include "cmsis.h"
#include "PortNames.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALT1    0x100
#define ALT2    0x200
#define ALT3    0x300
#define ALT4    0x400
#define ALT5    0x500
#define ALT6    0x600
#define ALT7    0x700

#define ALTMASK 0x700

typedef enum {
#if defined GPIOA
  PORTA_0  = (PORTA << 4) + 0x00,
  PORTA_1  = (PORTA << 4) + 0x01,
  PORTA_2  = (PORTA << 4) + 0x02,
  PORTA_3  = (PORTA << 4) + 0x03,
  PORTA_4  = (PORTA << 4) + 0x04,
  PORTA_5  = (PORTA << 4) + 0x05,
  PORTA_6  = (PORTA << 4) + 0x06,
  PORTA_7  = (PORTA << 4) + 0x07,
  PORTA_8  = (PORTA << 4) + 0x08,
  PORTA_9  = (PORTA << 4) + 0x09,
  PORTA_10 = (PORTA << 4) + 0x0A,
  PORTA_11 = (PORTA << 4) + 0x0B,
  PORTA_12 = (PORTA << 4) + 0x0C,
  PORTA_13 = (PORTA << 4) + 0x0D,
  PORTA_14 = (PORTA << 4) + 0x0E,
  PORTA_15 = (PORTA << 4) + 0x0F,
#endif
#if defined GPIOB
  PORTB_0  = (PORTB << 4) + 0x10,
  PORTB_1  = (PORTB << 4) + 0x11,
  PORTB_2  = (PORTB << 4) + 0x12,
  PORTB_3  = (PORTB << 4) + 0x13,
  PORTB_4  = (PORTB << 4) + 0x14,
  PORTB_5  = (PORTB << 4) + 0x15,
  PORTB_6  = (PORTB << 4) + 0x16,
  PORTB_7  = (PORTB << 4) + 0x17,
  PORTB_8  = (PORTB << 4) + 0x18,
  PORTB_9  = (PORTB << 4) + 0x19,
  PORTB_10 = (PORTB << 4) + 0x1A,
  PORTB_11 = (PORTB << 4) + 0x1B,
  PORTB_12 = (PORTB << 4) + 0x1C,
  PORTB_13 = (PORTB << 4) + 0x1D,
  PORTB_14 = (PORTB << 4) + 0x1E,
  PORTB_15 = (PORTB << 4) + 0x1F,
#endif
#if defined GPIOC
  PORTC_0  = (PORTC << 4) + 0x20,
  PORTC_1  = (PORTC << 4) + 0x21,
  PORTC_2  = (PORTC << 4) + 0x22,
  PORTC_3  = (PORTC << 4) + 0x23,
  PORTC_4  = (PORTC << 4) + 0x24,
  PORTC_5  = (PORTC << 4) + 0x25,
  PORTC_6  = (PORTC << 4) + 0x26,
  PORTC_7  = (PORTC << 4) + 0x27,
  PORTC_8  = (PORTC << 4) + 0x28,
  PORTC_9  = (PORTC << 4) + 0x29,
  PORTC_10 = (PORTC << 4) + 0x2A,
  PORTC_11 = (PORTC << 4) + 0x2B,
  PORTC_12 = (PORTC << 4) + 0x2C,
  PORTC_13 = (PORTC << 4) + 0x2D,
  PORTC_14 = (PORTC << 4) + 0x2E,
  PORTC_15 = (PORTC << 4) + 0x2F,
#endif
#if defined GPIOD
  PORTD_0  = (PORTD << 4) + 0x30,
  PORTD_1  = (PORTD << 4) + 0x31,
  PORTD_2  = (PORTD << 4) + 0x32,
  PORTD_3  = (PORTD << 4) + 0x33,
  PORTD_4  = (PORTD << 4) + 0x34,
  PORTD_5  = (PORTD << 4) + 0x35,
  PORTD_6  = (PORTD << 4) + 0x36,
  PORTD_7  = (PORTD << 4) + 0x37,
  PORTD_8  = (PORTD << 4) + 0x38,
  PORTD_9  = (PORTD << 4) + 0x39,
  PORTD_10 = (PORTD << 4) + 0x3A,
  PORTD_11 = (PORTD << 4) + 0x3B,
  PORTD_12 = (PORTD << 4) + 0x3C,
  PORTD_13 = (PORTD << 4) + 0x3D,
  PORTD_14 = (PORTD << 4) + 0x3E,
  PORTD_15 = (PORTD << 4) + 0x3F,
#endif
#if defined GPIOE
  PORTE_0  = (PORTE << 4) + 0x40,
  PORTE_1  = (PORTE << 4) + 0x41,
  PORTE_2  = (PORTE << 4) + 0x42,
  PORTE_3  = (PORTE << 4) + 0x43,
  PORTE_4  = (PORTE << 4) + 0x44,
  PORTE_5  = (PORTE << 4) + 0x45,
  PORTE_6  = (PORTE << 4) + 0x46,
  PORTE_7  = (PORTE << 4) + 0x47,
  PORTE_8  = (PORTE << 4) + 0x48,
  PORTE_9  = (PORTE << 4) + 0x49,
  PORTE_10 = (PORTE << 4) + 0x4A,
  PORTE_11 = (PORTE << 4) + 0x4B,
  PORTE_12 = (PORTE << 4) + 0x4C,
  PORTE_13 = (PORTE << 4) + 0x4D,
  PORTE_14 = (PORTE << 4) + 0x4E,
  PORTE_15 = (PORTE << 4) + 0x4F,
#endif
#if defined GPIOF
  PORTF_0  = (PORTF << 4) + 0x50,
  PORTF_1  = (PORTF << 4) + 0x51,
  PORTF_2  = (PORTF << 4) + 0x52,
  PORTF_3  = (PORTF << 4) + 0x53,
  PORTF_4  = (PORTF << 4) + 0x54,
  PORTF_5  = (PORTF << 4) + 0x55,
  PORTF_6  = (PORTF << 4) + 0x56,
  PORTF_7  = (PORTF << 4) + 0x57,
  PORTF_8  = (PORTF << 4) + 0x58,
  PORTF_9  = (PORTF << 4) + 0x59,
  PORTF_10 = (PORTF << 4) + 0x5A,
  PORTF_11 = (PORTF << 4) + 0x5B,
  PORTF_12 = (PORTF << 4) + 0x5C,
  PORTF_13 = (PORTF << 4) + 0x5D,
  PORTF_14 = (PORTF << 4) + 0x5E,
  PORTF_15 = (PORTF << 4) + 0x5F,
#endif
#if defined GPIOG
  PORTG_0  = (PORTG << 4) + 0x60,
  PORTG_1  = (PORTG << 4) + 0x61,
  PORTG_2  = (PORTG << 4) + 0x62,
  PORTG_3  = (PORTG << 4) + 0x63,
  PORTG_4  = (PORTG << 4) + 0x64,
  PORTG_5  = (PORTG << 4) + 0x65,
  PORTG_6  = (PORTG << 4) + 0x66,
  PORTG_7  = (PORTG << 4) + 0x67,
  PORTG_8  = (PORTG << 4) + 0x68,
  PORTG_9  = (PORTG << 4) + 0x69,
  PORTG_10 = (PORTG << 4) + 0x6A,
  PORTG_11 = (PORTG << 4) + 0x6B,
  PORTG_12 = (PORTG << 4) + 0x6C,
  PORTG_13 = (PORTG << 4) + 0x6D,
  PORTG_14 = (PORTG << 4) + 0x6E,
  PORTG_15 = (PORTG << 4) + 0x6F,
#endif
#if defined GPIOH
  PORTH_0  = (PORTH << 4) + 0x70,
  PORTH_1  = (PORTH << 4) + 0x71,
  PORTH_2  = (PORTH << 4) + 0x72,
  PORTH_3  = (PORTH << 4) + 0x73,
  PORTH_4  = (PORTH << 4) + 0x74,
  PORTH_5  = (PORTH << 4) + 0x75,
  PORTH_6  = (PORTH << 4) + 0x76,
  PORTH_7  = (PORTH << 4) + 0x77,
  PORTH_8  = (PORTH << 4) + 0x78,
  PORTH_9  = (PORTH << 4) + 0x79,
  PORTH_10 = (PORTH << 4) + 0x7A,
  PORTH_11 = (PORTH << 4) + 0x7B,
  PORTH_12 = (PORTH << 4) + 0x7C,
  PORTH_13 = (PORTH << 4) + 0x7D,
  PORTH_14 = (PORTH << 4) + 0x7E,
  PORTH_15 = (PORTH << 4) + 0x7F,
#endif
#if defined GPIOI
  PORTI_0  = (PORTI << 4) + 0x80,
  PORTI_1  = (PORTI << 4) + 0x81,
  PORTI_2  = (PORTI << 4) + 0x82,
  PORTI_3  = (PORTI << 4) + 0x83,
  PORTI_4  = (PORTI << 4) + 0x84,
  PORTI_5  = (PORTI << 4) + 0x85,
  PORTI_6  = (PORTI << 4) + 0x86,
  PORTI_7  = (PORTI << 4) + 0x87,
  PORTI_8  = (PORTI << 4) + 0x88,
  PORTI_9  = (PORTI << 4) + 0x89,
  PORTI_10 = (PORTI << 4) + 0x8A,
  PORTI_11 = (PORTI << 4) + 0x8B,
  PORTI_12 = (PORTI << 4) + 0x8C,
  PORTI_13 = (PORTI << 4) + 0x8D,
  PORTI_14 = (PORTI << 4) + 0x8E,
  PORTI_15 = (PORTI << 4) + 0x8F,
#endif
  //INT_ADC_BASE = PIN_NAME_INT_ANALOG_BASE,
  /* ADC internal channels */
  ADC_PINS_BASE = 0x100,
  ADC_TEMP,
  ADC_VREF,
/* pin names specific to the variant */
#if __has_include("PinNamesVar.h")
#include "PinNamesVar.h"
#endif
  NC = 0xFFFFFFFF
} PinName;

/* pin mode */
#define PIN_MODE_SHIFT        0
#define PIN_MODE_MASK         0x7
#define PIN_MODE_BITS         (0x7 << 0)
/* pin remap */
#define PIN_REMAP_SHIFT       3
#define PIN_REMAP_MASK        0x7F
#define PIN_REMAP_BITS        (0x7f << 3)
/* pin seed */
#define PIN_SPEED_SHIFT       10
#define PIN_SPEED_MASK        0x3
#define PIN_SPEED_BITS        (0x3 << 10)
/* pin output mode */
#define PIN_OUTPUT_MODE_SHIFT 12
#define PIN_OUTPUT_MODE_MASK  0x1
#define PIN_OUTPUT_MODE_BITS  (0x1 << 12)
/* pin pull_up_down */
#define PIN_PULL_STATE_SHIFT  13
#define PIN_PULL_STATE_MASK   0x3
#define PIN_PULL_STATE_BITS   (0x3 << 13)
#define PIN_INPUT_PU_BITS     (0x1 << 13)
#define PIN_INPUT_PD_BITS     (0x1 << 14)
/* pin af */
#define PIN_AF_SHIFT          15
#define PIN_AF_MASK           0xF
#define PIN_AF_BITS           (0xf << 15)
/* pin channel */
#define PIN_CHANNEL_SHIFT     19
#define PIN_CHANNEL_MASK      0x1F
#define PIN_CHANNEL_BITS      (0x1f << 19)
/* pin PWM channel-ON state */
#define PIN_CHON_SHIFT        24
#define PIN_CHON_MASK         0x1
#define PIN_CHON_BITS         (0x1 << 24)

/**
 * BIT[7:4] port number (0=PORTA, 1=PORTB, 2=PORTC, 3=PORTD, 4=PORTE)
 * BIT[3:0] pin number
 */
//#define GD_PORT_GET(X)  (port_to_gpio_reg(((uint32_t)(X) >> 4) & 0xF))
#define GD_PIN_GET(X)   (((uint32_t)(X) & 0xF))

/* get mode, speed, remap, output, pull state, af function, channel, and channel-ON of GPIO pin */
#define GD_PIN_MODE_GET(X)        ((X >> PIN_MODE_SHIFT) & PIN_MODE_MASK)
#define GD_PIN_SPEED_GET(X)       ((X >> PIN_SPEED_SHIFT) & PIN_SPEED_MASK)
#define GD_PIN_REMAP_GET(X)       ((X >> PIN_REMAP_SHIFT) & PIN_REMAP_MASK)
#define GD_PIN_OUTPUT_MODE_GET(X) ((X >> PIN_OUTPUT_MODE_SHIFT) & PIN_OUTPUT_MODE_MASK)
#define GD_PIN_PULL_STATE_GET(X)  ((X >> PIN_PULL_STATE_SHIFT) & PIN_PULL_STATE_MASK)
#define GD_PIN_AF_GET(X)          ((X >> PIN_AF_SHIFT) & PIN_AF_MASK)
#define GD_PIN_CHANNEL_GET(X)     ((X >> PIN_CHANNEL_SHIFT) & PIN_CHANNEL_MASK)
#define GD_PIN_CHON_GET(X)        ((X >> PIN_CHON_SHIFT) & PIN_CHON_MASK)

/**
 * This typedef is used to extend the PinMode typedef enum
 * in the ArduinoAPI, since they don't have constants
 */
typedef enum {
  OUTPUT_OPEN_DRAIN = 4,  // This was merged into Arduino API, so we know it will have this value in the future.
  INPUT_ANALOG,           // With the merge of both GD32 pin APIs, I believe we can get rid of this Analog define.
} PinModeExtension;

/* GPIO pull-up/pull-down/none */
enum {
  PIN_PUPD_NONE = 0,
  PIN_PUPD_PULLUP = 1,
  PIN_PUPD_PULLDOWN = 2,
};

/* GPIO output push-pull/open drain */
enum {
  PIN_OTYPE_PP = 0,
  PIN_OTYPE_OD = 1,
};

/* GPIO output speed type */
enum {
  PIN_SPEED_2MHZ = 0,
  PIN_SPEED_10MHZ = 1,
  PIN_SPEED_50MHZ = 3,
};

#define PIN_MODE_INPUT    0
#define PIN_MODE_OUTPUT   1
#define PIN_MODE_AF       2
#define PIN_MODE_ANALOG   3

/**
 * We need a way to insure we cleanly encode these bits
 * in a unified way. Using the definitions below makes sure all devices
 * are encoded the same way. This cleans up a lot of code.
 * Decoding happens later in a different function
 */
#define PIN_MODE_INPUT_FLOATING (PIN_MODE_INPUT)
#define PIN_MODE_INPUT_PU       (PIN_MODE_INPUT | PIN_INPUT_PU_BITS)
#define PIN_MODE_INPUT_PD       (PIN_MODE_INPUT | PIN_INPUT_PD_BITS)
#define PIN_MODE_OUT_OD         (PIN_MODE_OUTPUT | PIN_OUTPUT_MODE_BITS)
#define PIN_MODE_OUT_PP         (PIN_MODE_OUTPUT)
#define PIN_MODE_AF_OD          (PIN_MODE_AF | PIN_OUTPUT_MODE_BITS)
#define PIN_MODE_AF_PP          (PIN_MODE_AF)

#define GD_PIN_FUNC_ANALOG_CH(CHAN)   ((int)(PIN_MODE_ANALOG & PIN_MODE_MASK) | ((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | (((int)PIN_MODE_INPUT_FLOATING & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT))

#define GD_PIN_DATA(MODE, PUPD, AFN)  ((int)(MODE) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT) | ((AFN & PIN_AF_MASK) << PIN_AF_SHIFT))
#define GD_PIN_DATA_EXT(MODE, PUPD, AFN, CHAN, CHON) \
                                      ((int)(MODE) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT) | \
                                      ((AFN & PIN_AF_MASK) << PIN_AF_SHIFT) | \
                                      ((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | \
                                      ((CHON & PIN_CHON_MASK) << PIN_CHON_SHIFT))
#define GD_PIN_DATA_REMAP(MODE, PUPD, REMAP)  ((int)(MODE & PIN_MODE_MASK) | \
                                              ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT ) | \
                                              ((REMAP & PIN_REMAP_MASK) << PIN_REMAP_SHIFT))
#define GD_PIN_DATA_EXT_REMAP(MODE, PUPD, REMAP, CHAN, CHON) \
                                      ((int)(MODE) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT) | \
                                      ((REMAP & PIN_REMAP_MASK) << PIN_REMAP_SHIFT) | \
                                      ((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | \
                                      ((CHON & PIN_CHON_MASK) << PIN_CHON_SHIFT))

#ifdef __cplusplus
}
#endif

#endif
