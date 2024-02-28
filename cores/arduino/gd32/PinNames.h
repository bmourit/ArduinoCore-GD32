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

#ifdef __cplusplus
extern "C" {
#endif

#define ALT1    				0x100
#define ALT2    				0x200
#define ALT3    				0x300
#define ALT4    				0x400
#define ALT5    				0x500
#define ALT6    				0x600
#define ALT7    				0x700

#define ALTMASK 				0x700

#define PIN_NAME_INT_ANALOG_BASE	0x1000

typedef enum {
#if defined GPIOA
	PORTA_0  = 0x00,
	PORTA_1  = 0x01,
	PORTA_2  = 0x02,
	PORTA_3  = 0x03,
	PORTA_4  = 0x04,
	PORTA_5  = 0x05,
	PORTA_6  = 0x06,
	PORTA_7  = 0x07,
	PORTA_8  = 0x08,
	PORTA_9  = 0x09,
	PORTA_10 = 0x0A,
	PORTA_11 = 0x0B,
	PORTA_12 = 0x0C,
	PORTA_13 = 0x0D,
	PORTA_14 = 0x0E,
	PORTA_15 = 0x0F,
#endif
#if defined GPIOB
	PORTB_0  = 0x10,
	PORTB_1  = 0x11,
	PORTB_2  = 0x12,
	PORTB_3  = 0x13,
	PORTB_4  = 0x14,
	PORTB_5  = 0x15,
	PORTB_6  = 0x16,
	PORTB_7  = 0x17,
	PORTB_8  = 0x18,
	PORTB_9  = 0x19,
	PORTB_10 = 0x1A,
	PORTB_11 = 0x1B,
	PORTB_12 = 0x1C,
	PORTB_13 = 0x1D,
	PORTB_14 = 0x1E,
	PORTB_15 = 0x1F,
#endif
#if defined GPIOC
	PORTC_0  = 0x20,
	PORTC_1  = 0x21,
	PORTC_2  = 0x22,
	PORTC_3  = 0x23,
	PORTC_4  = 0x24,
	PORTC_5  = 0x25,
	PORTC_6  = 0x26,
	PORTC_7  = 0x27,
	PORTC_8  = 0x28,
	PORTC_9  = 0x29,
	PORTC_10 = 0x2A,
	PORTC_11 = 0x2B,
	PORTC_12 = 0x2C,
	PORTC_13 = 0x2D,
	PORTC_14 = 0x2E,
	PORTC_15 = 0x2F,
#endif
#if defined GPIOD
	PORTD_0  = 0x30,
	PORTD_1  = 0x31,
	PORTD_2  = 0x32,
	PORTD_3  = 0x33,
	PORTD_4  = 0x34,
	PORTD_5  = 0x35,
	PORTD_6  = 0x36,
	PORTD_7  = 0x37,
	PORTD_8  = 0x38,
	PORTD_9  = 0x39,
	PORTD_10 = 0x3A,
	PORTD_11 = 0x3B,
	PORTD_12 = 0x3C,
	PORTD_13 = 0x3D,
	PORTD_14 = 0x3E,
	PORTD_15 = 0x3F,
#endif
#if defined GPIOE
	PORTE_0  = 0x40,
	PORTE_1  = 0x41,
	PORTE_2  = 0x42,
	PORTE_3  = 0x43,
	PORTE_4  = 0x44,
	PORTE_5  = 0x45,
	PORTE_6  = 0x46,
	PORTE_7  = 0x47,
	PORTE_8  = 0x48,
	PORTE_9  = 0x49,
	PORTE_10 = 0x4A,
	PORTE_11 = 0x4B,
	PORTE_12 = 0x4C,
	PORTE_13 = 0x4D,
	PORTE_14 = 0x4E,
	PORTE_15 = 0x4F,
#endif
#if defined GPIOF
	PORTF_0  = 0x50,
	PORTF_1  = 0x51,
	PORTF_2  = 0x52,
	PORTF_3  = 0x53,
	PORTF_4  = 0x54,
	PORTF_5  = 0x55,
	PORTF_6  = 0x56,
	PORTF_7  = 0x57,
	PORTF_8  = 0x58,
	PORTF_9  = 0x59,
	PORTF_10 = 0x5A,
	PORTF_11 = 0x5B,
	PORTF_12 = 0x5C,
	PORTF_13 = 0x5D,
	PORTF_14 = 0x5E,
	PORTF_15 = 0x5F,
#endif
#if defined GPIOG
	PORTG_0  = 0x60,
	PORTG_1  = 0x61,
	PORTG_2  = 0x62,
	PORTG_3  = 0x63,
	PORTG_4  = 0x64,
	PORTG_5  = 0x65,
	PORTG_6  = 0x66,
	PORTG_7  = 0x67,
	PORTG_8  = 0x68,
	PORTG_9  = 0x69,
	PORTG_10 = 0x6A,
	PORTG_11 = 0x6B,
	PORTG_12 = 0x6C,
	PORTG_13 = 0x6D,
	PORTG_14 = 0x6E,
	PORTG_15 = 0x6F,
#endif
#if defined GPIOH
	PORTH_0  = 0x70,
	PORTH_1  = 0x71,
	PORTH_2  = 0x72,
	PORTH_3  = 0x73,
	PORTH_4  = 0x74,
	PORTH_5  = 0x75,
	PORTH_6  = 0x76,
	PORTH_7  = 0x77,
	PORTH_8  = 0x78,
	PORTH_9  = 0x79,
	PORTH_10 = 0x7A,
	PORTH_11 = 0x7B,
	PORTH_12 = 0x7C,
	PORTH_13 = 0x7D,
	PORTH_14 = 0x7E,
	PORTH_15 = 0x7F,
#endif
#if defined GPIOI
	PORTI_0  = 0x80,
	PORTI_1  = 0x81,
	PORTI_2  = 0x82,
	PORTI_3  = 0x83,
	PORTI_4  = 0x84,
	PORTI_5  = 0x85,
	PORTI_6  = 0x86,
	PORTI_7  = 0x87,
	PORTI_8  = 0x88,
	PORTI_9  = 0x89,
	PORTI_10 = 0x8A,
	PORTI_11 = 0x8B,
	PORTI_12 = 0x8C,
	PORTI_13 = 0x8D,
	PORTI_14 = 0x8E,
	PORTI_15 = 0x8F,
#endif
	INT_ADC_BASE = PIN_NAME_INT_ANALOG_BASE,
	/* ADC internal channels */
	ADC_TEMP,
	ADC_VREF,

/* pin names specific to the variant */
#if __has_include("PinNamesVar.h")
#include "PinNamesVar.h"
#endif

	NC = (int)0xFFFFFFFF
} PinName;

/* NOTE: Is this used anywhere? Do we plan to? Should we remove? */
/* struct gpio_function {
	uint8_t gpio_mode : 3;		//[0:2]
	uint8_t remap : 7;		//[3:9]
	uint8_t gpio_speed : 2;		//[10:11]
	uint8_t out_pp_od : 1;		//[12]
	uint8_t pull_states : 2;	//[13:14]
	uint8_t af_num : 4;		//[15:18]
	uint8_t channel_num : 5;	//[19:23]
	uint8_t reverse_state : 1;	//[24]
	uint8_t reserved : 8;		//[25:31]
}; */

/*  void pin_function(PinName pin, int function);
	configure the mode, pull, speed, af, remap, and PWM Channel-ON function of pins
	the parameter function contains the configuration information,show as below
	bit 0:2   gpio mode input / output / af / analog
	bit 3:9   remap
	bit 10:11 gpio speed
	bit 12    output push-pull / open drain
	bit 13:14 no pull, pull-up, pull-down
	bit 15:18 channel af function
	bit 19:23 channel of adc/timer/dac
	bit 24    PWM channel-ON
	bit 25:31 reserved
*/

/* pin mode */
#define PIN_MODE_SHIFT 0,
#define PIN_MODE_MASK  0x7,
#define PIN_MODE_BITS (PIN_MODE_MASK << PIN_MODE_SHIFT)
/* pin remap */
#define PIN_REMAP_SHIFT 3,
#define PIN_REMAP_MASK  0x7F,
#define PIN_REMAP_BITS (PIN_REMAP_MASK << PIN_REMAP_SHIFT)
/* pin seed */
#define PIN_SPEED_SHIFT 10,
#define PIN_SPEED_MASK  0x3,
#define PIN_SPEED_BITS (PIN_SPEED_MASK << PIN_SPEED_SHIFT)
/* pin output mode */
#define PIN_OUTPUT_MODE_SHIFT 12,
#define PIN_OUTPUT_MODE_MASK  0x1,
#define PIN_OUTPUT_MODE_BITS (PIN_OUTPUT_MODE_MASK << PIN_OUTPUT_MODE_SHIFT)
/* pin pull_up_down */
#define PIN_PULL_STATE_SHIFT 13,
#define PIN_PULL_STATE_MASK  0x3,
#define PIN_PULL_STATE_BITS (PIN_PULL_STATE_MASK << PIN_PULL_STATE_SHIFT)
#define PIN_INPUT_PU_BITS	(0x1 << PIN_PULL_STATE_SHIFT)
#define PIN_INPUT_PD_BITS	(0x1 << 14)
/* pin af */
#define PIN_AF_SHIFT 15,
#define PIN_AF_MASK  0xF,
#define PIN_AF_BITS (PIN_AF_MASK << PIN_AF_SHIFT)
/* pin channel */
#define PIN_CHANNEL_SHIFT 19,
#define PIN_CHANNEL_MASK  0x1F,
#define PIN_CHANNEL_BITS (PIN_CHANNEL_MASK << PIN_CHANNEL_SHIFT)
/* pin PWM channel-ON state */
#define PIN_CHON_SHIFT 24,
#define PIN_CHON_MASK  0x1,
#define PIN_CHON_BITS (PIN_CHON_MASK << PIN_CHON_SHIFT)


/*
 * BIT[7:4] port number (0=PORTA, 1=PORTB, 2=PORTC, 3=PORTD, 4=PORTE)
 * BIT[3:0] pin number
 */
#define GD_PORT_GET(X) (((uint32_t)(X) >> 4) & 0xF)
#define GD_PIN_GET(X)  (((uint32_t)(X) & 0xF))

/* get mode, speed, remap, output, pull state, af function, channel, and channel-ON of GPIO pin */
#define GD_PIN_MODE_GET(X)			((X >> PIN_MODE_SHIFT) & PIN_MODE_MASK)
#define GD_PIN_SPEED_GET(X)			((X >> PIN_SPEED_SHIFT) & PIN_SPEED_MASK)
#define GD_PIN_REMAP_GET(X)			((X >> PIN_REMAP_SHIFT) & PIN_REMAP_MASK)
#define GD_PIN_OUTPUT_MODE_GET(X)	((X >> PIN_OUTPUT_MODE_SHIFT) & PIN_OUTPUT_MODE_MASK)
#define GD_PIN_PULL_STATE_GET(X)	((X >> PIN_PULL_STATE_SHIFT) & PIN_PULL_STATE_MASK)
#define GD_PIN_AF_GET(X)			((X >> PIN_AF_SHIFT) & PIN_AF_MASK)
#define GD_PIN_CHANNEL_GET(X)		((X >> PIN_CHANNEL_SHIFT) & PIN_CHANNEL_MASK)
#define GD_PIN_CHON_GET(X)			((X >> PIN_CHON_SHIFT) & PIN_CHON_MASK)

/*
/* This typedef is used to extend the PinMode typedef enum
/* in the ArduinoAPI, since they don't have constants
 */
typedef enum {
	OUTPUT_OPEN_DRAIN = 4,	// This was merged into Arduino API, so we know it will have this value in the future.
	INPUT_ANALOG,		// With the merge of both GD32 pin APIs, I believe we can get rid of this Analog define.
} PinModeExtension;

/* GPIO pull-up/pull-down/none */
enum {
	PIN_PUPD_NONE		= 0,
	PIN_PUPD_PULLUP		= 1,
	PIN_PUPD_PULLDOWN	= 2,
};

/* GPIO output push-pull/open drain */
enum {
	PIN_OTYPE_PP	= 0,
	PIN_OTYPE_OD	= 1,
};

/* GPIO output speed type */
enum {
	PIN_SPEED_2MHZ		= 0,
	PIN_SPEED_10MHZ		= 1,
	PIN_SPEED_50MHZ		= 3,
};

//#define GD_PIN_FUNCTION1(MODE, REMAP) ((int)(MODE & PIN_MODE_MASK) | ((REMAP & PIN_REMAP_MASK) << PIN_REMAP_SHIFT))
//#define GD_PIN_FUNCTION2(MODE, REMAP, CHN, CHON) ((int)(MODE & PIN_MODE_MASK) | ((REMAP & PIN_REMAP_MASK) << PIN_REMAP_SHIFT) |\
												  ((CHN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | ((CHON & PIN_CHON_MASK) << PIN_CHON_SHIFT))
//#define GD_PIN_FUNCTION3(MODE, ODPP, AFN) ((int)(MODE & PIN_MODE_MASK) | ((ODPP & PIN_OUTPUT_MODE_MASK) << PIN_OUTPUT_MODE_SHIFT) | ((AFN & PIN_AF_MASK) << PIN_AF_SHIFT))
//#define GD_PIN_FUNCTION4(MODE, ODPP, PUPD, AFN) ((int)(MODE & PIN_MODE_MASK) | ((ODPP & PIN_OUTPUT_MODE_MASK) << PIN_OUTPUT_MODE_SHIFT) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT )  | ((AFN & PIN_AF_MASK) << PIN_AF_SHIFT))
//#define GD_PIN_FUNCTION5(MODE, ODPP, PUPD, REMAP) ((int)(MODE & PIN_MODE_MASK) | ((ODPP & PIN_OUTPUT_MODE_MASK) << PIN_OUTPUT_MODE_SHIFT) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT )  | ((REMAP & PIN_REMAP_MASK) << PIN_REMAP_SHIFT))
#define GD_PIN_FUNC_ANALOG_CH(CHAN) ((int)(PIN_MODE_ANALOG & PIN_MODE_MASK) | \
								((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | \
								(((int)PIN_MODE_INPUT_FLOATING & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT))
//#define GD_PIN_FUNC_PWM(CHN, AFN) ((PIN_MODE_AF & PIN_MODE_MASK) | \
								((PIN_OTYPE_PP & PIN_OUTPUT_MODE_MASK) << PIN_OUTPUT_MODE_SHIFT) | \
								((PIN_SPEED_50MHZ & PIN_SPEED_MASK) << PIN_SPEED_SHIFT) | \
								((CHN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | \
								((AFN & PIN_AF_MASK) << PIN_AF_SHIFT) | \
								((PIN_PUPD_NONE & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT))

#define GD_PIN_DATA(MODE, PUPD, AFN)	((int)(MODE) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT) | ((AFN & PIN_AF_MASK) << PIN_AF_SHIFT))
#define GD_PIN_DATA_EXT(MODE, PUPD, AFN, CHAN, CHON) \
								((int)(MODE) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT) | \
									((AFN & PIN_AF_MASK) << PIN_AF_SHIFT) | \
									((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | \
									((CHON & PIN_CHON_MASK) << PIN_CHON_SHIFT))
#define GD_PIN_DATA_REMAP(MODE, PUPD, REMAP)	((int)(MODE & PIN_MODE_MASK) | \
								    ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT ) | \
									((REMAP & PIN_REMAP_MASK) << PIN_REMAP_SHIFT))
#define GD_PIN_DATA_EXT_REMAP(MODE, PUPD, REMAP, CHAN, CHON) \
								((int)(MODE) | ((PUPD & PIN_PULL_STATE_MASK) << PIN_PULL_STATE_SHIFT) | \
									((REMAP & PIN_REMAP_MASK) << PIN_REMAP_SHIFT) | \
									((CHAN & PIN_CHANNEL_MASK) << PIN_CHANNEL_SHIFT) | \
									((CHON & PIN_CHON_MASK) << PIN_CHON_SHIFT))
typedef enum {
	PIN_MODE_INPUT = 0,
	PIN_MODE_OUTPUT = 1,
	PIN_MODE_AF = 2,
	PIN_MODE_ANALOG = 3,
} GDPinModeFunction;

/*
 * We need a way to insure we cleanly encode these bits
 * in a unified way. Using the definitions below makes sure all devices
 * are encoded the same way. This cleans up a lot of code.
 * Decoding happens later in a different function
 */
#define PIN_MODE_INPUT 			(PIN_MODE_INPUT)
#define PIN_MODE_INPUT_FLOATING		(PIN_MODE_INPUT)
#define PIN_MODE_INPUT_PU		(PIN_MODE_INPUT | PIN_INPUT_PU_BITS)
#define PIN_MODE_INPUT_PD		(PIN_MODE_INPUT | PIN_INPUT_PD_BITS)
#define PIN_MODE_OUT_OD			(PIN_MODE_OUTPUT | PIN_OUTPUT_MODE_BITS)
#define PIN_MODE_OUT_PP 		(PIN_MODE_OUTPUT)
#define PIN_MODE_AF_OD 			(PIN_MODE_AF | PIN_OUTPUT_MODE_BITS)
#define PIN_MODE_AF_PP 			(PIN_MODE_AF)
#define PIN_MODE_ANALOG			(PIN_MODE_ANALOG)

#ifdef __cplusplus
}
#endif

#endif
