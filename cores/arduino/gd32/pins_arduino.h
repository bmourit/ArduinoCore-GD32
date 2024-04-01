/*
  Copyright (c) 2011 Arduino.  All right reserved.

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
#ifndef _PINS_ARDUINO_H_
#define _PINS_ARDUINO_H_
#include <stdbool.h>
#include <stdlib.h>
#include "variant.h"
#include "PinNames.h"
#include "PortNames.h"

/* Arduino digital pins alias */
enum {
	D0,   D1,   D2,   D3,   D4,   D5,   D6,   D7,   D8,   D9,
	D10,  D11,  D12,  D13,  D14,  D15,  D16,  D17,  D18,  D19,
	D20,  D21,  D22,  D23,  D24,  D25,  D26,  D27,  D28,  D29,
	D30,  D31,  D32,  D33,  D34,  D35,  D36,  D37,  D38,  D39,
	D40,  D41,  D42,  D43,  D44,  D45,  D46,  D47,  D48,  D49,
	D50,  D51,  D52,  D53,  D54,  D55,  D56,  D57,  D58,  D59,
	D60,  D61,  D62,  D63,  D64,  D65,  D66,  D67,  D68,  D69,
	D70,  D71,  D72,  D73,  D74,  D75,  D76,  D77,  D78,  D79,
	D80,  D81,  D82,  D83,  D84,  D85,  D86,  D87,  D88,  D89,
	D90,  D91,  D92,  D93,  D94,  D95,  D96,  D97,  D98,  D99,
	D100, D101, D102, D103, D104, D105, D106, D107, D108, D109,
	D110, D111, D112, D113, D114, D115, D116, D117, D118, D119,
	D120, D121, D122, D123, D124, D125, D126, D127, D128, D129,
	D130, D131, D132, D133, D134, D135, D136, D137, D138, D139,
	D140, D141, D142, D143, D144, D145, D146, D147, D148, D149,
	D150, D151, D152, D153, D154, D155, D156, D157, D158, D159,
	D160, D161, D162, D163, D164, D165, D166, D167, D168, D169,
	D170, D171, D172, D173, D174, D175, D176, D177, D178, D179,
	D180, D181, D182, D183, D184, D185, D186, D187, D188, D189,
	D190, D191, DMAX
};

/* configure periphera pins */
static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#ifdef HAVE_I2C
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;
#endif

#ifdef HAVE_I2C1
static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;
#endif

#ifdef HAVE_I2C2
static const uint8_t SDA2 = PIN_WIRE2_SDA;
static const uint8_t SCL2 = PIN_WIRE2_SCL;
#endif

/* the PIN_SERIAL_TX/RX definitions point to the default Serial's pins */
#if DEFAULT_HWSERIAL_INSTANCE == 1
#define PIN_SERIAL_RX       SERIAL0_RX
#define PIN_SERIAL_TX       SERIAL0_TX
#elif DEFAULT_HWSERIAL_INSTANCE == 2
#define PIN_SERIAL_RX       SERIAL1_RX
#define PIN_SERIAL_TX       SERIAL1_TX
#elif DEFAULT_HWSERIAL_INSTANCE == 3
#define PIN_SERIAL_RX       SERIAL2_RX
#define PIN_SERIAL_TX       SERIAL2_TX
#elif DEFAULT_HWSERIAL_INSTANCE == 4
#define PIN_SERIAL_RX       SERIAL3_RX
#define PIN_SERIAL_TX       SERIAL3_TX
#elif DEFAULT_HWSERIAL_INSTANCE == 5
#define PIN_SERIAL_RX       SERIAL4_RX
#define PIN_SERIAL_TX       SERIAL4_TX
#endif

static const uint8_t TX = PIN_SERIAL_TX;
static const uint8_t RX = PIN_SERIAL_RX;

#if defined(SERIAL0_RX)
static const uint8_t RX0 = SERIAL0_RX;
#endif

#if defined(SERIAL0_TX)
static const uint8_t TX0 = SERIAL0_TX;
#endif

#if defined(SERIAL1_RX)
static const uint8_t RX1 = SERIAL1_RX;
#endif

#if defined(SERIAL1_TX)
static const uint8_t TX1 = SERIAL1_TX;
#endif

#if defined(SERIAL2_RX)
static const uint8_t RX2 = SERIAL2_RX;
#endif

#if defined(SERIAL2_TX)
static const uint8_t TX2 = SERIAL2_TX;
#endif

#if defined(SERIAL3_RX)
static const uint8_t RX3 = SERIAL3_RX;
#endif

#if defined(SERIAL3_TX)
static const uint8_t TX3 = SERIAL3_TX;
#endif

#if defined(SERIAL4_RX)
static const uint8_t RX4 = SERIAL4_RX;
#endif

#if defined(SERIAL4_TX)
static const uint8_t TX4 = SERIAL4_TX;
#endif

/* configure analog pins */
#define PIN_NUM_MASK		0xff
/*
 * Analog base pin number
 * easyily check if a pin number is an analog pin:
 * ((pin & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE)
 * if true then pin is an analog pin number
 */
#define ANALOG_PIN_NUM_BASE	0xc0

/*
 * Pin number Analog index
 * allows to retrieve the pin number index of an analog pin
 * when used with the above base
 * (p & ANALOG_PIN_NUM_INDEX)
 */
#define ANALOG_PIN_NUM_INDEX	0x3f

#ifndef ANALOG_PINS_NUM
#define ANALOG_PINS_NUM 0
#endif

/* these need to go last, so after the analog pins */
#define ANALOG_INTERNAL_START      (ANALOG_PIN_NUM_BASE + ANALOG_PINS_NUM)

/* ADC internal channels */
/* used for analogRead() */
#if defined(ADC_CHANNEL_TEMPSENSOR) || defined(ADC_CHANNEL_TEMPSENSOR_ADC1)
#define ATEMP        (ANALOG_INTERNAL_START)
#endif
#ifdef ADC_CHANNEL_VREFINT
#define AVREF        (ANALOG_INTERNAL_START + 1)
#endif

#if ANALOG_PINS_NUM > 0
static const uint8_t A0 = ANALOG_PIN_NUM_BASE;
#endif
#if ANALOG_PINS_NUM > 1
static const uint8_t A1 = ANALOG_PIN_NUM_BASE + 1;
#endif
#if ANALOG_PINS_NUM > 2
static const uint8_t A2 = ANALOG_PIN_NUM_BASE + 2;
#endif
#if ANALOG_PINS_NUM > 3
static const uint8_t A3 = ANALOG_PIN_NUM_BASE + 3;
#endif
#if ANALOG_PINS_NUM > 4
static const uint8_t A4 = ANALOG_PIN_NUM_BASE + 4;
#endif
#if ANALOG_PINS_NUM > 5
static const uint8_t A5 = ANALOG_PIN_NUM_BASE + 5;
#endif
#if ANALOG_PINS_NUM > 6
static const uint8_t A6 = ANALOG_PIN_NUM_BASE + 6;
#endif
#if ANALOG_PINS_NUM > 7
static const uint8_t A7 = ANALOG_PIN_NUM_BASE + 7;
#endif
#if ANALOG_PINS_NUM > 8
static const uint8_t A8 = ANALOG_PIN_NUM_BASE + 8;
#endif
#if ANALOG_PINS_NUM > 9
static const uint8_t A9 = ANALOG_PIN_NUM_BASE + 9;
#endif
#if ANALOG_PINS_NUM > 10
static const uint8_t A10 = ANALOG_PIN_NUM_BASE + 10;
#endif
#if ANALOG_PINS_NUM > 11
static const uint8_t A11 = ANALOG_PIN_NUM_BASE + 11;
#endif
#if ANALOG_PINS_NUM > 12
static const uint8_t A12 = ANALOG_PIN_NUM_BASE + 12;
#endif
#if ANALOG_PINS_NUM > 13
static const uint8_t A13 = ANALOG_PIN_NUM_BASE + 13;
#endif
#if ANALOG_PINS_NUM > 14
static const uint8_t A14 = ANALOG_PIN_NUM_BASE + 14;
#endif
#if ANALOG_PINS_NUM > 15
static const uint8_t A15 = ANALOG_PIN_NUM_BASE + 15;
#endif
#if ANALOG_PINS_NUM > 16
static const uint8_t A16 = ANALOG_PIN_NUM_BASE + 16;
#endif
#if ANALOG_PINS_NUM > 17
static const uint8_t A17 = ANALOG_PIN_NUM_BASE + 17;
#endif
#if ANALOG_PINS_NUM > 18
static const uint8_t A18 = ANALOG_PIN_NUM_BASE + 18;
#endif
#if ANALOG_PINS_NUM > 19
static const uint8_t A19 = ANALOG_PIN_NUM_BASE + 19;
#endif
#if ANALOG_PINS_NUM > 20
static const uint8_t A20 = ANALOG_PIN_NUM_BASE + 20;
#endif
#if ANALOG_PINS_NUM > 21
static const uint8_t A21 = ANALOG_PIN_NUM_BASE + 21;
#endif
#if ANALOG_PINS_NUM > 22
static const uint8_t A22 = ANALOG_PIN_NUM_BASE + 22;
#endif
#if ANALOG_PINS_NUM > 23
static const uint8_t A23 = ANALOG_PIN_NUM_BASE + 23;
#endif
#if ANALOG_PINS_NUM > 24
static const uint8_t A24 = ANALOG_PIN_NUM_BASE + 24;
#endif
#if ANALOG_PINS_NUM > 25
static const uint8_t A25 = ANALOG_PIN_NUM_BASE + 25;
#endif
#if ANALOG_PINS_NUM > 26
static const uint8_t A26 = ANALOG_PIN_NUM_BASE + 26;
#endif
#if ANALOG_PINS_NUM > 27
static const uint8_t A27 = ANALOG_PIN_NUM_BASE + 27;
#endif
#if ANALOG_PINS_NUM > 28
static const uint8_t A28 = ANALOG_PIN_NUM_BASE + 28;
#endif
#if ANALOG_PINS_NUM > 29
static const uint8_t A29 = ANALOG_PIN_NUM_BASE + 29;
#endif
#if ANALOG_PINS_NUM > 30
static const uint8_t A30 = ANALOG_PIN_NUM_BASE + 20;
#endif
#if ANALOG_PINS_NUM > 31
static const uint8_t A31 = ANALOG_PIN_NUM_BASE + 31;
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern const PinName digital_pins[];
extern const uint32_t analog_pins[];

extern const uint32_t gpio_port[];
extern const uint32_t gpio_pin[];

#define NOT_INTERRUPT	NC

/* Convert a digital pin to a PinName */
#if ANALOG_PINS_NUM > 0
#define DIGITAL_TO_PINNAME(p)	((((uint32_t)(p) & PIN_NUM_MASK) < DIGITAL_PINS_NUM) ? \
					(PinName)(digital_pins[(uint32_t)(p) & PIN_NUM_MASK] | ((p) & ALTMASK)) : \
                                       	(((uint32_t)(p) & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE) && \
                                       	(((uint32_t)(p) & PIN_NUM_MASK) < ANALOG_INTERNAL_START) ? \
                                       	(PinName)(digital_pins[analog_pins[(p) & ANALOG_PIN_NUM_INDEX]] | ((p) & ALTMASK)) : NC)
#else
#define DIGITAL_TO_PINNAME(p)	((((uint32_t)(p) & PIN_NUM_MASK) < DIGITAL_PINS_NUM) ? \
					(PinName)(digital_pins[(uint32_t)(p) & PIN_NUM_MASK] | ((p) & ALTMASK)) : NC)
#endif /* ANALOG_PINS_NUM > 0 */

/* Convert a PinName to a digital pin */
uint32_t PinName_to_digital(PinName p);

/* Convert an analog pin to a digital pin */
#if ANALOG_PINS_NUM > 0
/* used by analogRead api to have A0 == 0 */
/* non-contiguous analog pins definition in digital_pins array */
#define ANALOG_PINS_TO_DIGITAL(p)	( \
                                     	(((uint32_t)(p) & PIN_NUM_MASK) < ANALOG_PINS_NUM) ? \
                                     	analog_pins[(uint32_t)(p) & PIN_NUM_MASK] | ((uint32_t)(p) & ALTMASK) : \
                                     	(((uint32_t)(p) & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE) && \
                                     	(((uint32_t)(p) & PIN_NUM_MASK) < ANALOG_INTERNAL_START) ? \
                                     	analog_pins[(p) & ANALOG_PIN_NUM_INDEX] | ((uint32_t)(p) & ALTMASK) : (uint32_t)NC)
#else /* No analog pin defined */
#define ANALOG_PINS_TO_DIGITAL(p)	(DIGITAL_PINS_NUM)
#endif /* ANALOG_PINS_NUM > 0 */

/* Convert an analog pin to a PinName */
PinName analog_pin_to_PinName(uint32_t pin);

/* All pins could manage EXTI */
#define DIGITAL_PIN_VALID(p)		(DIGITAL_TO_PINNAME(p) != NC)
#define DIGITAL_PIN_TO_INT(p)		(DIGITAL_PIN_VALID(p) ? p : NOT_INTERRUPT)

#define DIGITAL_PIN_I2C(p)		(pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_I2C_SDA) || \
						pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_I2C_SCL))
#define DIGITAL_PIN_PWM(p)		(pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_PWM))
#define DIGITAL_PIN_SERIAL(p)		(pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_UART_RX) || \
						pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_UART_TX))
#define DIGITAL_PIN_SPI(p)		(pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_MOSI) || \
						pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_MISO) || \
						pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_SCLK) || \
						pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_SSEL))


#define DIGITAL_PIN_TO_PORT(p)		((GD_PORT_GET(DIGITAL_TO_PINNAME(p)) < GPIO_PORT_NUM) ? gpio_port[GD_PORT_GET(DIGITAL_TO_PINNAME(p))] : (uint32_t )NULL)
#define DIGITAL_PIN_TO_BIT_MASK(p)	((uint16_t)(1 << GD_PIN_GET(DIGITAL_TO_PINNAME(p))))

#define ANALOG_PINS_TO_BIT(p)		((uint16_t)(1 << GD_PIN_GET(DIGITAL_TO_PINNAME(p))))
#define PORT_OUTPUT_REG(p)		(GPIO_OCTL(p))
#define PORT_INPUT_REG(p)		(GPIO_ISTAT(p))
#define PORT_SET_REG(p)			(GPIO_BOP(p))
#if defined(GD32F4xx)
/* GPIO register definitions */
#define PORT_CLEAR_REG(p)		(GPIO_BOP(p))
#else
#define PORT_CLEAR_REG(p)		(GPIO_BC(p))
#endif

#if defined(GD32F30x)
#define PORT_CTL_REG(p)			(GPIO_CTL0(p))
#else
#define PORT_CTL_REG(p)			(GPIO_CTL(p))
#endif
#define PORT_CFG_REG(p)			(PORT_CTL_REG(p))

/* some pins could be duplicated in digital_pins[] */
/* return first occurence of linked PinName (PYx) */
#define ANALOG_PINS_FIRST_LINK(p) 	(PinName_to_digital(DIGITAL_TO_PINNAME(p)))

/* ensure pin is not one of the serial pins */
#if defined(PIN_SERIAL_RX) && defined(PIN_SERIAL_TX)
#define PIN_IS_SERIAL(p)	((ANALOG_PINS_FIRST_LINK(p) == (PIN_SERIAL_RX & PIN_NUM_MASK)) || \
					(ANALOG_PINS_FIRST_LINK(p) == (PIN_SERIAL_TX & PIN_NUM_MASK)))
#endif
/* Convert a digital pin to an analog pin */
bool pin_in_analog_pins(uint32_t pin);
uint32_t digital_pin_to_analog(uint32_t pin);

/* macros for Arduino compatibility */
#define digitalPinToPinName(P) (DIGITAL_TO_PINNAME(P))
#define digitalPinToPort(P) (DIGITAL_PIN_TO_PORT(P))
#define digitalPinToBitMask(P) (DIGITAL_PIN_TO_BIT_MASK(P))
#define portInputRegister(PORT) (&PORT_INPUT_REG(PORT))
#define portOutputRegister(PORT) (&PORT_OUTPUT_REG(PORT))
#define portModeRegister(PORT) (&PORT_CTL_REG(PORT))

#ifdef __cplusplus
}
#endif

/* default values can be overridden by variants */
#ifndef ADC_RESOLUTION
  #define ADC_RESOLUTION	10
#endif
#ifndef PWM_RESOLUTION
  #define PWM_RESOLUTION	8
#endif

#define DAC_RESOLUTION		12

#ifndef PWM_FREQUENCY
  #define PWM_FREQUENCY		1000    // 1000µS = 1ms = 1kHz
#endif
#ifndef PWM_MAX_DUTY_CYCLE
  #define PWM_MAX_DUTY_CYCLE	4095
#endif

#endif /*_PINS_ARDUINO_H_*/
