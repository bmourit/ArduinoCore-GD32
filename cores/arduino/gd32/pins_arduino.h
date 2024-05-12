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

#include "arduino_analog_pins.h"
#include "arduino_digital_pins.h"

/**
 * Pin number mask
 * allows to retrieve the pin number without ALTx
 */
#define PIN_NUM_MASK      0xff

/* undefined pin */
#define PIN_NOT_DEFINED   DIGITAL_PINS_NUM

/* spi pins */
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS      10
#endif
#ifndef PIN_SPI_SS1
  #define PIN_SPI_SS1     4
#endif
#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI    11
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO    12
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK     13
#endif

static const uint32_t SS = PIN_SPI_SS;
static const uint32_t SS1 = PIN_SPI_SS1;
static const uint32_t MOSI = PIN_SPI_MOSI;
static const uint32_t MISO = PIN_SPI_MISO;
static const uint32_t SCK = PIN_SPI_SCK;

/* i2c pins */
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA    14
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL    15
#endif

#ifdef HAVE_I2C1
#ifndef PIN_WIRE1_SDA
  #define PIN_WIRE1_SDA   16
#endif
#ifndef PIN_WIRE1_SCL
  #define PIN_WIRE1_SCL   17
#endif
#endif

#ifdef HAVE_I2C2
#ifndef PIN_WIRE2_SDA
  #define PIN_WIRE2_SDA   18
#endif
#ifndef PIN_WIRE2_SCL
  #define PIN_WIRE2_SCL   19
#endif
#endif

#ifdef HAVE_I2C
static const uint32_t SDA = PIN_WIRE_SDA;
static const uint32_t SCL = PIN_WIRE_SCL;
#endif

#ifdef HAVE_I2C1
static const uint32_t SDA1 = PIN_WIRE1_SDA;
static const uint32_t SCL1 = PIN_WIRE1_SCL;
#endif

#ifdef HAVE_I2C2
static const uint32_t SDA2 = PIN_WIRE2_SDA;
static const uint32_t SCL2 = PIN_WIRE2_SCL;
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern const PinName digital_pins[];
extern const uint32_t analog_pins[];

/* TODO: Move these GPIO defines to gpio related file */
/*------------------------------------------------------------------------------*/
extern const uint32_t gpio_port[];
extern const uint32_t gpio_pin[];
/*------------------------------------------------------------------------------*/

#define NOT_INTERRUPT   NC

/* Convert a digital pin to a PinName */
#if ANALOG_PINS_NUM > 0
#define DIGITAL_TO_PINNAME(p) ((((uint32_t)(p) & PIN_NUM_MASK) < DIGITAL_PINS_NUM) ? \
                                (PinName)(digital_pins[(uint32_t)(p) & PIN_NUM_MASK] | ((p) & ALTMASK)) : \
                                (((uint32_t)(p) & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE) && \
                                (((uint32_t)(p) & PIN_NUM_MASK) < ANALOG_INTERNAL_START) ? \
                                (PinName)(digital_pins[analog_pins[(p) & ANALOG_PIN_NUM_INDEX]] | ((p) & ALTMASK)) : NC)
#else
#define DIGITAL_TO_PINNAME(p) ((((uint32_t)(p) & PIN_NUM_MASK) < DIGITAL_PINS_NUM) ? \
                                (PinName)(digital_pins[(uint32_t)(p) & PIN_NUM_MASK] | ((p) & ALTMASK)) : NC)
#endif /* ANALOG_PINS_NUM > 0 */

/* Convert a PinName to a digital pin */
uint32_t PinName_to_digital(PinName p);

/* Convert an analog pin to a digital pin */
#if ANALOG_PINS_NUM > 0
/* used by analogRead api to have A0 == 0 */
/* non-contiguous analog pins definition in digital_pins array */
#define ANALOG_PIN_TO_DIGITAL(p) ((((uint32_t)(p) & PIN_NUM_MASK) < ANALOG_PINS_NUM) ? \
                                    analog_pins[(uint32_t)(p) & PIN_NUM_MASK] | ((uint32_t)(p) & ALTMASK) : \
                                    (((uint32_t)(p) & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE) && \
                                    (((uint32_t)(p) & PIN_NUM_MASK) < ANALOG_INTERNAL_START) ? \
                                    analog_pins[(p) & ANALOG_PIN_NUM_INDEX] | ((uint32_t)(p) & ALTMASK) : (uint32_t)NC)
#else /* No analog pin defined */
#define ANALOG_PIN_TO_DIGITAL(p)   (DIGITAL_PINS_NUM)
#endif /* ANALOG_PINS_NUM > 0 */

/* Convert an analog pin to a PinName */
PinName analog_pin_to_PinName(uint32_t pin);

/* All pins could manage EXTI */
#define DIGITAL_PIN_TO_INT(p)       (DIGITAL_PIN_VALID(p) ? p : NOT_INTERRUPT)
#define DIGITAL_PIN_VALID(p)        (DIGITAL_TO_PINNAME(p) != NC)
#define DIGITAL_PIN_I2C(p)          (pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_I2C_SDA) || \
                                      pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_I2C_SCL))
#define DIGITAL_PIN_PWM(p)          (pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_PWM))
#define DIGITAL_PIN_SERIAL(p)       (pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_UART_RX) || \
                                      pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_UART_TX))
#define DIGITAL_PIN_SPI(p)          (pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_MOSI) || \
                                      pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_MISO) || \
                                      pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_SCLK) || \
                                      pin_in_pinmap(DIGITAL_TO_PINNAME(p), PinMap_SPI_SSEL))

#define DIGITAL_PIN_TO_PORT(p)      ((GD_PORT_GET(DIGITAL_TO_PINNAME(p)) < GPIO_PORT_NUM) ? gpio_port[GD_PORT_GET(DIGITAL_TO_PINNAME(p))] : (uint32_t )NULL)
#define DIGITAL_PIN_TO_BIT_MASK(p)  (gpio_pin[GD_PIN_GET(DIGITAL_TO_PINNAME(p))])
#define ANALOG_PINS_TO_BIT(p)       (gpio_pin[GD_PIN_GET(DIGITAL_TO_PINNAME(p))])

#define PORT_OUTPUT_REG(port)       (GPIO_OCTL(port))
#define PORT_INPUT_REG(port)        (GPIO_ISTAT(port))
#define PORT_SET_REG(port)          (GPIO_BOP(port))
#if defined(GD32F4xx)
/* GPIO register definitions */
#define PORT_CLEAR_REG(port)        (GPIO_BOP(port))
#else
#define PORT_CLEAR_REG(port)        (GPIO_BC(port))
#endif

#if defined(GD32F30x) || defined(GD32F10x)
// Config bits split across two registers:
// CTL0: pin 0..7
// CTL1: pin 8..15
// Return only CTL0
#define PORT_CTL_REG(port)          (GPIO_CTL0(port))
#else
#define PORT_CTL_REG(port)          (GPIO_CTL(port))
#endif
#define PORT_CFG_REG(port)          (PORT_CTL_REG(port))

/* since some pins could be duplicated in digital_pins[] */
/* return the first occurence of linked PinName (PYx) */
#define ANALOG_PINS_FIRST_LINK(p)   (PINNAME_TO_DIGITAL(DIGITAL_TO_PINNAME(p)))

/* ensure pin is not one of the serial pins */
#if defined(PIN_SERIAL_RX) && defined(PIN_SERIAL_TX)
#define PIN_IS_SERIAL(p)            ((ANALOG_PINS_FIRST_LINK(p) == (PIN_SERIAL_RX & PIN_NUM_MASK)) || \
                                      (ANALOG_PINS_FIRST_LINK(p) == (PIN_SERIAL_TX & PIN_NUM_MASK)))
#endif

/* Convert a digital pin to an analog pin */
bool pin_is_analog_pin(uint32_t pin);
uint32_t digital_pin_to_analog(uint32_t pin);

/* macros for Arduino compatibility */
#define digitalPinToPinName(P)      (DIGITAL_TO_PINNAME(P))
#define digitalPinToPort(P)         (DIGITAL_PIN_TO_PORT(P))
#define digitalPinToBitMask(P)      (DIGITAL_PIN_TO_BIT_MASK(P))
#define portInputRegister(PORT)     (&PORT_INPUT_REG(PORT))
#define portOutputRegister(PORT)    (&PORT_OUTPUT_REG(PORT))
#define portModeRegister(PORT)      (&PORT_CTL_REG(PORT))

#ifdef __cplusplus
}
#endif

/* default values can be overridden by variants */
#ifndef ADC_RESOLUTION
  #define ADC_RESOLUTION        10
#endif
#ifndef PWM_RESOLUTION
  #define PWM_RESOLUTION        8
#endif
#ifndef DAC_RESOLUTION
  #define DAC_RESOLUTION        12
#endif
#ifndef PWM_FREQUENCY
  #define PWM_FREQUENCY         1000  // 1000µS = 1ms = 1kHz
#endif
#ifndef PWM_MAX_DUTY_CYCLE
  #define PWM_MAX_DUTY_CYCLE    4095
#endif

#endif /*_PINS_ARDUINO_H_*/
