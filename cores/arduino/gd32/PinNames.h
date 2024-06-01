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

#include "PortNames.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis.h"

#define ALT1    0x100
#define ALT2    0x200
#define ALT3    0x300
#define ALT4    0x400
#define ALT5    0x500
#define ALT6    0x600
#define ALT7    0x700

#define ALTMASK 0x700

/* analog internal */
#define PINNAME_ANALOG_INT  0x1000

#define PINNAME_MASK        0xff

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
  PORTB_0  = (PORTB << 4) + 0x00,
  PORTB_1  = (PORTB << 4) + 0x01,
  PORTB_2  = (PORTB << 4) + 0x02,
  PORTB_3  = (PORTB << 4) + 0x03,
  PORTB_4  = (PORTB << 4) + 0x04,
  PORTB_5  = (PORTB << 4) + 0x05,
  PORTB_6  = (PORTB << 4) + 0x06,
  PORTB_7  = (PORTB << 4) + 0x07,
  PORTB_8  = (PORTB << 4) + 0x08,
  PORTB_9  = (PORTB << 4) + 0x09,
  PORTB_10 = (PORTB << 4) + 0x0A,
  PORTB_11 = (PORTB << 4) + 0x0B,
  PORTB_12 = (PORTB << 4) + 0x0C,
  PORTB_13 = (PORTB << 4) + 0x0D,
  PORTB_14 = (PORTB << 4) + 0x0E,
  PORTB_15 = (PORTB << 4) + 0x0F,
#endif
#if defined GPIOC
  PORTC_0  = (PORTC << 4) + 0x00,
  PORTC_1  = (PORTC << 4) + 0x01,
  PORTC_2  = (PORTC << 4) + 0x02,
  PORTC_3  = (PORTC << 4) + 0x03,
  PORTC_4  = (PORTC << 4) + 0x04,
  PORTC_5  = (PORTC << 4) + 0x05,
  PORTC_6  = (PORTC << 4) + 0x06,
  PORTC_7  = (PORTC << 4) + 0x07,
  PORTC_8  = (PORTC << 4) + 0x08,
  PORTC_9  = (PORTC << 4) + 0x09,
  PORTC_10 = (PORTC << 4) + 0x0A,
  PORTC_11 = (PORTC << 4) + 0x0B,
  PORTC_12 = (PORTC << 4) + 0x0C,
  PORTC_13 = (PORTC << 4) + 0x0D,
  PORTC_14 = (PORTC << 4) + 0x0E,
  PORTC_15 = (PORTC << 4) + 0x0F,
#endif
#if defined GPIOD
  PORTD_0  = (PORTD << 4) + 0x00,
  PORTD_1  = (PORTD << 4) + 0x01,
  PORTD_2  = (PORTD << 4) + 0x02,
  PORTD_3  = (PORTD << 4) + 0x03,
  PORTD_4  = (PORTD << 4) + 0x04,
  PORTD_5  = (PORTD << 4) + 0x05,
  PORTD_6  = (PORTD << 4) + 0x06,
  PORTD_7  = (PORTD << 4) + 0x07,
  PORTD_8  = (PORTD << 4) + 0x08,
  PORTD_9  = (PORTD << 4) + 0x09,
  PORTD_10 = (PORTD << 4) + 0x0A,
  PORTD_11 = (PORTD << 4) + 0x0B,
  PORTD_12 = (PORTD << 4) + 0x0C,
  PORTD_13 = (PORTD << 4) + 0x0D,
  PORTD_14 = (PORTD << 4) + 0x0E,
  PORTD_15 = (PORTD << 4) + 0x0F,
#endif
#if defined GPIOE
  PORTE_0  = (PORTE << 4) + 0x00,
  PORTE_1  = (PORTE << 4) + 0x01,
  PORTE_2  = (PORTE << 4) + 0x02,
  PORTE_3  = (PORTE << 4) + 0x03,
  PORTE_4  = (PORTE << 4) + 0x04,
  PORTE_5  = (PORTE << 4) + 0x05,
  PORTE_6  = (PORTE << 4) + 0x06,
  PORTE_7  = (PORTE << 4) + 0x07,
  PORTE_8  = (PORTE << 4) + 0x08,
  PORTE_9  = (PORTE << 4) + 0x09,
  PORTE_10 = (PORTE << 4) + 0x0A,
  PORTE_11 = (PORTE << 4) + 0x0B,
  PORTE_12 = (PORTE << 4) + 0x0C,
  PORTE_13 = (PORTE << 4) + 0x0D,
  PORTE_14 = (PORTE << 4) + 0x0E,
  PORTE_15 = (PORTE << 4) + 0x0F,
#endif
#if defined GPIOF
  PORTF_0  = (PORTF << 4) + 0x00,
  PORTF_1  = (PORTF << 4) + 0x01,
  PORTF_2  = (PORTF << 4) + 0x02,
  PORTF_3  = (PORTF << 4) + 0x03,
  PORTF_4  = (PORTF << 4) + 0x04,
  PORTF_5  = (PORTF << 4) + 0x05,
  PORTF_6  = (PORTF << 4) + 0x06,
  PORTF_7  = (PORTF << 4) + 0x07,
  PORTF_8  = (PORTF << 4) + 0x08,
  PORTF_9  = (PORTF << 4) + 0x09,
  PORTF_10 = (PORTF << 4) + 0x0A,
  PORTF_11 = (PORTF << 4) + 0x0B,
  PORTF_12 = (PORTF << 4) + 0x0C,
  PORTF_13 = (PORTF << 4) + 0x0D,
  PORTF_14 = (PORTF << 4) + 0x0E,
  PORTF_15 = (PORTF << 4) + 0x0F,
#endif
#if defined GPIOG
  PORTG_0  = (PORTG << 4) + 0x00,
  PORTG_1  = (PORTG << 4) + 0x01,
  PORTG_2  = (PORTG << 4) + 0x02,
  PORTG_3  = (PORTG << 4) + 0x03,
  PORTG_4  = (PORTG << 4) + 0x04,
  PORTG_5  = (PORTG << 4) + 0x05,
  PORTG_6  = (PORTG << 4) + 0x06,
  PORTG_7  = (PORTG << 4) + 0x07,
  PORTG_8  = (PORTG << 4) + 0x08,
  PORTG_9  = (PORTG << 4) + 0x09,
  PORTG_10 = (PORTG << 4) + 0x0A,
  PORTG_11 = (PORTG << 4) + 0x0B,
  PORTG_12 = (PORTG << 4) + 0x0C,
  PORTG_13 = (PORTG << 4) + 0x0D,
  PORTG_14 = (PORTG << 4) + 0x0E,
  PORTG_15 = (PORTG << 4) + 0x0F,
#endif
#if defined GPIOH
  PORTH_0  = (PORTH << 4) + 0x00,
  PORTH_1  = (PORTH << 4) + 0x01,
  PORTH_2  = (PORTH << 4) + 0x02,
  PORTH_3  = (PORTH << 4) + 0x03,
  PORTH_4  = (PORTH << 4) + 0x04,
  PORTH_5  = (PORTH << 4) + 0x05,
  PORTH_6  = (PORTH << 4) + 0x06,
  PORTH_7  = (PORTH << 4) + 0x07,
  PORTH_8  = (PORTH << 4) + 0x08,
  PORTH_9  = (PORTH << 4) + 0x09,
  PORTH_10 = (PORTH << 4) + 0x0A,
  PORTH_11 = (PORTH << 4) + 0x0B,
  PORTH_12 = (PORTH << 4) + 0x0C,
  PORTH_13 = (PORTH << 4) + 0x0D,
  PORTH_14 = (PORTH << 4) + 0x0E,
  PORTH_15 = (PORTH << 4) + 0x0F,
#endif
#if defined GPIOI
  PORTI_0  = (PORTI << 4) + 0x00,
  PORTI_1  = (PORTI << 4) + 0x01,
  PORTI_2  = (PORTI << 4) + 0x02,
  PORTI_3  = (PORTI << 4) + 0x03,
  PORTI_4  = (PORTI << 4) + 0x04,
  PORTI_5  = (PORTI << 4) + 0x05,
  PORTI_6  = (PORTI << 4) + 0x06,
  PORTI_7  = (PORTI << 4) + 0x07,
  PORTI_8  = (PORTI << 4) + 0x08,
  PORTI_9  = (PORTI << 4) + 0x09,
  PORTI_10 = (PORTI << 4) + 0x0A,
  PORTI_11 = (PORTI << 4) + 0x0B,
  PORTI_12 = (PORTI << 4) + 0x0C,
  PORTI_13 = (PORTI << 4) + 0x0D,
  PORTI_14 = (PORTI << 4) + 0x0E,
  PORTI_15 = (PORTI << 4) + 0x0F,
#endif
  /* ADC internal channels */
  ADC_PINS_BASE = PINNAME_ANALOG_INT,
#if defined(ADC_CHANNEL_TEMPSENSOR) || defined(ADC_CHANNEL_TEMPSENSOR_ADC1)
  ADC_TEMP,
#endif
#if defined(ADC_CHANNEL_VREFINT)
  ADC_VREF,
#endif
  ADC_START_ANALOG,
/* pin names specific to the variant */
#if __has_include("PinNamesVar.h")
#include "PinNamesVar.h"
#endif
  NC = (int)0xFFFFFFFF
} PinName;

/* pin mode */
#define PIN_MODE_SHIFT        0
#define PIN_MODE_MASK         0x7
#define PIN_MODE_BITS         (PIN_MODE_MASK << PIN_MODE_SHIFT)

/* pin output mode */
#define PIN_OUTPUT_OD_SHIFT   3
#define PIN_OUTPUT_OD_MASK    0x1
#define PIN_OUTPUT_OD_BITS    (PIN_OUTPUT_OD_MASK << PIN_OUTPUT_OD_SHIFT)

/* pin pull_up_down */
#define PIN_PULL_UD_SHIFT     4
#define PIN_PULL_UD_MASK      0x3
#define PIN_PULL_UD_BITS      (PIN_PULL_UD_MASK << PIN_PULL_UD_SHIFT)

/* pin speed */
#define PIN_SPEED_SHIFT       6
#define PIN_SPEED_MASK        0x3
#define PIN_SPEED_BITS        (PIN_SPEED_MASK << PIN_SPEED_SHIFT)

/* pin af */
#define PIN_AF_SHIFT          8
#define PIN_AF_MASK           0x7F
#define PIN_AF_BITS           (PIN_AF_MASK << PIN_AF_SHIFT)

/* pin channel */
#define PIN_CHANNEL_SHIFT     15
#define PIN_CHANNEL_MASK      0x1F
#define PIN_CHANNEL_BITS      (PIN_CHANNEL_MASK << PIN_CHANNEL_SHIFT)

/* pin PWM channel-ON state */
#define PIN_CHON_SHIFT        20
#define PIN_CHON_MASK         0x1
#define PIN_CHON_BITS         (PIN_CHON_MASK << PIN_CHON_SHIFT)

#define PIN_ANALOG_CTL_SHIFT  21
#define PIN_ANALOG_CTL_MASK   0x1
#define PIN_ANALOG_CTL_BITS   (PIN_ANALOG_CTL_MASK << PIN_ANALOG_CTL_SHIFT)

/**
 * BIT[7:4] port number (0=PORTA, 1=PORTB, 2=PORTC, 3=PORTD, 4=PORTE)
 * BIT[3:0] pin number
 */
#define GD_PORT_GET(X)  (((uint32_t)(X) >> 4) & 0xF)
#define GD_PIN_GET(X)   (((uint32_t)(X) & 0xF))

/* get mode, speed, remap, output, pull state, af function, channel, and channel-ON of GPIO pin function */
#define GD_PIN_MODE_GET(X)        (((X) >> PIN_MODE_SHIFT) & PIN_MODE_MASK)
#define GD_PIN_OUTPUT_OD_GET(X)   (((X) >> PIN_OUTPUT_OD_SHIFT) & PIN_OUTPUT_OD_MASK)
#define GD_PIN_PULL_UD_GET(X)     (((X) >> PIN_PULL_UD_SHIFT) & PIN_PULL_UD_MASK)
#define GD_PIN_SPEED_GET(X)       (((X) >> PIN_SPEED_SHIFT) & PIN_SPEED_MASK)
#define GD_PIN_AF_GET(X)          (((X) >> PIN_AF_SHIFT) & PIN_AF_MASK)
#define GD_PIN_CHANNEL_GET(X)     (((X) >> PIN_CHANNEL_SHIFT) & PIN_CHANNEL_MASK)
#define GD_PIN_CHON_GET(X)        (((X) >> PIN_CHON_SHIFT) & PIN_CHON_MASK)
#define GD_PIN_ANALOG_CTL_GET(X)  (((X) >> PIN_ANALOG_CTL_SHIFT) & PIN_ANALOG_CTL_MASK)
#define GD_PIN_MODE(X)            ((GD_PIN_OUTPUT_OD_GET((X)) << 4) | (GD_PIN_MODE_GET((X)) & (~PIN_OUTPUT_OD_BITS)))

#define GD_PIN_DATA(MODE_OD, PUPD, AFN)   ((int)(MODE_OD) | \
                                          ((PUPD & PIN_PULL_UD_MASK) << PIN_PULL_UD_SHIFT) | \
                                          ((AFN & PIN_AF_MASK) << PIN_AF_SHIFT))

#define GD_PIN_DATA_EXT(MODE_OD, PUPD, AFN, CHAN, CHON) \
                                          ((int)(MODE_OD) | ((PUPD & PIN_PULL_UD_MASK) << PIN_PULL_UD_SHIFT) | \
                                          ((AFN & PIN_AF_MASK) << PIN_AF_SHIFT) | \
                                          ((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | \
                                          ((CHON & PIN_CHON_MASK) << PIN_CHON_SHIFT))
/**
 * This typedef is used to extend the PinMode typedef enum
 * in the ArduinoAPI, since they don't have constants
 */
//typedef enum {
//  OUTPUT_OPEN_DRAIN = 4,  // This was merged into Arduino API, so we know it will have this value in the future.
//  INPUT_ANALOG,           // With the merge of both GD32 pin APIs, I believe we can get rid of this Analog define.
//} PinModeExtension;

/* GPIO pull-up/pull-down/none */
#define PIN_PUPD_NONE       0
#define PIN_PUPD_PULLUP     1
#define PIN_PUPD_PULLDOWN   2

/* GPIO output push-pull/open drain */
#define PIN_OTYPE_PP    0
#define PIN_OTYPE_OD    1

/* GPIO output speed type */
#define PIN_SPEED_2MHZ  0
#define PIN_SPEED_10MHZ 1
#define PIN_SPEED_50MHZ 3

typedef enum {
  GD_PIN_INPUT = 0,
  GD_PIN_OUTPUT = 1,
  GD_PIN_AF = 2,
  GD_PIN_ANALOG = 3,
} gd_pin_mode_t;

/**
 * We need a way to insure we cleanly encode these bits
 * in a unified way. Using the definitions below makes sure all devices
 * are encoded the same way. This cleans up a lot of code.
 * Decoding happens later in a different function
 */
#define GD_MODE_INPUT          (GD_PIN_INPUT)
#define GD_MODE_OUT_OD         (GD_PIN_OUTPUT | PIN_OUTPUT_OD_BITS)
#define GD_MODE_OUT_PP         (GD_PIN_OUTPUT)
#define GD_MODE_AF_OD          (GD_PIN_AF | PIN_OUTPUT_OD_BITS)
#define GD_MODE_AF_PP          (GD_PIN_AF)
#define GD_MODE_ANALOG         (GD_PIN_ANALOG)

#define GD_VALID_PINNAME(X)   (GD_PORT_GET(X) <= LastPort)
#define GD_GPIO_PIN(X)        ((uint16_t)(1 << GD_PIN_GET(X)))

#define GD_PIN_FUNC_ANALOG_CH(CHAN)       ((int)(GD_PIN_ANALOG & PIN_MODE_MASK) | ((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | (((int)PIN_PUPD_NONE & PIN_PULL_UD_MASK) << PIN_PULL_UD_SHIFT))

#ifdef __cplusplus
}
#endif

#endif
