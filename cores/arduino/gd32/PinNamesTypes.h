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
#ifndef _PINNAMESTYPES_H
#define _PINNAMESTYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*  GD PIN data as used in pin_function is coded on 32 bits as below
 *   [2:0]  Function (like in MODER reg) : Input / Output / Alt / Analog
 *     [3]  Output Push-Pull / Open Drain (as in OTYPER reg)
 *   [5:4]  as in PUPDR reg: No Pull, Pull-up, Pull-Down
 *   [7:6]  Reserved for speed config (as in OSPEEDR), but not used yet
 *  [14:8]  Alternate Num (as in AFRL/AFRG reg)
 * [19:15]  Channel (Analog/Timer specific)
 *    [20]  Inverted (Analog/Timer specific)
 *    [21]  Analog ADC control - Only valid for specific families
 *    [22]  Analog channel bank B - Only valid for specific families
 * [32:23]  Reserved
 */

/* pin function mode input, output, alt, analog */
#define PIN_FUNC_MASK   0x07
#define PIN_FUNC_SHIFT  0
#define PIN_FUNC_BITS   (PIN_FUNC_MASK << PIN_FUNC_SHIFT)

/* pin output push-pull, open drain */
#define PIN_PPOD_MASK   0x01
#define PIN_PPOD_SHIFT  3
#define PIN_PPOD_BITS   (PIN_OPOD_MASK << PIN_OPOD_SHIFT)

/* pin no pull, pull-up, pull-down */
#define PIN_PUPD_MASK   0x03
#define PIN_PUPD_SHIFT  4
#define PIN_PUPD_BITS   (PIN_PUPD_MASK << PIN_PUPD_SHIFT)

/* pin speed */
#define PIN_SPEED_MASK  0x03
#define PIN_SPEED_SHIFT 6
#define PIN_SPEED_BITS  (PIN_SPEED_MASK << PIN_SPEED_SHIFT)

/* pin alternate function */
#define PIN_AF_MASK     0x7F
#define PIN_AF_SHIFT    8
#define PIN_AF_BITS     (PIN_AF_MASK << PIN_AF_SHIFT)

/* pin channel */
#define PIN_CHAN_MASK   0x1F
#define PIN_CHAN_SHIFT  15
#define PIN_CHAN_BIT    (PIN_CHAN_MASK << PIN_CHAN_SHIFT)

/* pin inversion */
#define PIN_INV_MASK    0x01
#define PIN_INV_SHIFT   20
#define PIN_INV_BIT     (PIN_INV_MASK << PIN_INV_SHIFT)

/* pin analog adc control */
#define PIN_AADC_MASK   0x01
#define PIN_AADC_SHIFT  21
#define PIN_AADC_BIT    (PIN_ANLG_MASK << PIN_ANLG_SHIFT)

/* analog channel bank B */
#define PIN_AN_CHBK_B_MASK  0x01
#define PIN_AN_CHBK_B_SHIFT 22
#define PIN_AN_CHBK_B_BIT   (PIN_AN_CHBK_B_MASK << PIN_AN_CHBK_B_SHIFT)

/* BIT[7:4] port number (0=PORTA, 1=PORTB, 2=PORTC, 3=PORTD, 4=PORTE)
   BIT[3:0] pin number */
#define GD_PORT_GET(X) (((uint32_t)(X) >> 4) & 0xF)
#define GD_PIN_GET(X)  (((uint32_t)(X) & 0xF))

/* Get mode,speed,remap,output,pull state,af function,channel and channel-ON of GPIO pin */
#define GD_PIN_FUNC_GET(X)          (((X) >> PIN_FUNC_SHIFT) & PIN_FUNC_MASK)
#define GD_PIN_PPOD_GET(X)          (((X) >> PIN_PPOD_SHIFT) & PIN_PPOD_MASK)
#define GD_PIN_PUPD_GET(X)          (((X) >> PIN_PUPD_SHIFT) & PIN_PUPD_MASK)
#define GD_PIN_SPEED_GET(X)         (((X) >> PIN_SPEED_SHIFT) & PIN_SPEED_MASK)
#define GD_PIN_AF_GET(X)            (((X) >> PIN_AF_SHIFT) & PIN_AF_MASK)
#define GD_PIN_CHAN_GET(X)          (((X) >> PIN_CHAN_SHIFT) & PIN_CHAN_MASK)
#define GD_PIN_INV_GET(X)           (((X) >> PIN_INV_SHIFT) & PIN_INV_MASK)
#define GD_PIN_AADC_GET(X)          (((X) >> PIN_AADC_SHIFT) & PIN_AADC_MASK)
#define GD_PIN_AN_CHBK_B_GET(X)     (((X) >> PIN_AN_CHBK_B_SHIFT) & PIN_AN_CHBK_B_MASK)
#define GD_PIN_MODE(X)              ((GD_PIN_PPOD_GET((X)) < 4) | \
                                     (GD_PIN_FUNC_GET((X)) & (~PIN_PPOD_BITS)))

#define GD_PIN_DEFINE(FUNC_OD, PUPD, PAF)    ((int)(FUNC_OD) | \
                                        ((PUPD & PIN_PUPD_MASK) << PIN_PUPD_SHIFT) |\
                                        ((PAF & PIN_AF_MASK) << PIN_AF_SHIFT))

#define GD_PIN_DEFINE_EXT(FUNC_OD, PUPD, PAF, CHAN, INV) \
                                        ((int)(FUNC_OD) | \
                                            ((PUPD & PIN_PUPD_MASK) << PIN_PUPD_SHIFT) | \
                                            ((PAF & PIN_AF_MASK) << PIN_AF_SHIFT) | \
                                            ((CHAN & PIN_CHAN_MASK) << PIN_CHAN_SHIFT) | \
                                            ((INV & PIN_INV_MASK) << PIN_INV_SHIFT))

typedef enum {
    GD_PIN_INPUT = 0,
    GD_PIN_OUTPUT = 1,
    GD_PIN_ALTFUNC = 2,
    GD_PIN_ANALOG = 3,
} GdPinFunction;

#define PIN_MODE_INPUT      (GD_PIN_INPUT)
#define PIN_MODE_OUTPUT_PP  (GD_PIN_OUTPUT)
#define PIN_MODE_OUTPUT_OD  (GD_PIN_OUTPUT | PIN_PPOD_BITS)
#define PIN_MODE_AF_PP      (GD_PIN_ALTFUNC)
#define PIN_MODE_AF_OD      (GD_PIN_ALTFUNC | PIN_PPOD_BITS)
#define PIN_MODE_ANALOG     (GD_PIN_ANALOG)
#define PIN_MODE_AADC       (GD_PIN_ANALOG | PIN_AADC_BIT)
#define PIN_MODE_AN_CHBK_B  (GD_PIN_ANALOG | PIN_AN_CHBK_B_BIT)

// High nibble = port number (FirstPort <= PortName <= LastPort)
// Low nibble  = pin number
#define GD_PORT(X) (((uint32_t)(X) >> 4) & 0xF)
#define GD_PIN(X)  ((uint32_t)(X) & 0xF)
// Check PinName is valid: FirstPort <= PortName <= LastPort
// As FirstPort is equal to 0 and GD_PORT cast as an unsigned
// (GD_PORT(X) >= FirstPort)  is always true
//#define GD_VALID_PINNAME(X) ((GD_PORT(X) >= FirstPort) && (GD_PORT(X) <= LastPort))
#define GD_VALID_PINNAME(X) (GD_PORT(X) <= LastPort)

#define GD_GPIO_PIN(X) ((uint16_t)(1 << GD_PIN(X)))

#ifdef __cplusplus
}
#endif

#endif