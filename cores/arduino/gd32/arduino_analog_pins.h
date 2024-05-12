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

#ifndef _ARDUINO_ANALOG_PINS_H_
#define _ARDUINO_ANALOG_PINS_H_

#include "variant.h"
#include "PinNames.h"

/**
 * Analog base pin number
 * easyily check if a pin number is an analog pin:
 * ((pin & ANALOG_PIN_NUM_BASE) == ANALOG_PIN_NUM_BASE)
 * if true then pin is an analog pin number
 */
#define ANALOG_PIN_NUM_BASE   0xc0
/**
 * Pin number Analog index
 * allows to retrieve the pin number index of an analog pin
 * when used with the above base
 * (p & ANALOG_PIN_NUM_INDEX)
 */
#define ANALOG_PIN_NUM_INDEX    0x3f

#ifndef ANALOG_PINS_NUM
  #define ANALOG_PINS_NUM   0
#endif

/*
 * Pin number Analog index
 * allows to retrieve the pin number index of an analog pin
 * when used with the above base
 * (p & ANALOG_PIN_NUM_INDEX)
 */
#define ANALOG_PIN_NUM_INDEX    0x3f

/* these need to go last, so after the analog pins */
#define ANALOG_INTERNAL_START   (ANALOG_PIN_NUM_BASE + ANALOG_PINS_NUM)

/* ADC internal channels */
/* only used for analogRead() */
//#if defined(ADC_CHANNEL_TEMPSENSOR) || defined(ADC_CHANNEL_TEMPSENSOR_ADC0)
  #define ATEMP   (ANALOG_INTERNAL_START)
//#endif
//#ifdef ADC_CHANNEL_VREFINT
  #define AVREF   (ANALOG_INTERNAL_START + 1)
//#endif

#if ANALOG_PINS_NUM > 0
  #define PIN_A0  ANALOG_PIN_NUM_BASE
  static const uint8_t A0 = PIN_A0;
#if ANALOG_PINS_NUM > 1
  #define PIN_A1  (PIN_A0 + 1)
  static const uint8_t A1 = PIN_A1;
#endif
#if ANALOG_PINS_NUM > 2
  #define PIN_A2  (PIN_A1 + 1)
  static const uint8_t A2 = PIN_A2;
#endif
#if ANALOG_PINS_NUM > 3
  #define PIN_A3  (PIN_A2 + 1)
  static const uint8_t A3 = PIN_A3;
#endif
#if ANALOG_PINS_NUM > 4
  #define PIN_A4  (PIN_A3 + 1)
  static const uint8_t A4 = PIN_A4;
#endif
#if ANALOG_PINS_NUM > 5
  #define PIN_A5  (PIN_A4 + 1)
  static const uint8_t A5 = PIN_A5;
#endif
#if ANALOG_PINS_NUM > 6
  #define PIN_A6  (PIN_A5 + 1)
  static const uint8_t A6 = PIN_A6;
#endif
#if ANALOG_PINS_NUM > 7
  #define PIN_A7  (PIN_A6 + 1)
  static const uint8_t A7 = PIN_A7;
#endif
#if ANALOG_PINS_NUM > 8
  #define PIN_A8  (PIN_A7 + 1)
  static const uint8_t A8 = PIN_A8;
#endif
#if ANALOG_PINS_NUM > 9
  #define PIN_A9  (PIN_A8 + 1)
  static const uint8_t A9 = PIN_A9;
#endif
#if ANALOG_PINS_NUM > 10
  #define PIN_A10  (PIN_A9 + 1)
  static const uint8_t A10 = PIN_A10;
#endif
#if ANALOG_PINS_NUM > 11
  #define PIN_A11  (PIN_A10 + 1)
  static const uint8_t A11 = PIN_A11;
#endif
#if ANALOG_PINS_NUM > 12
  #define PIN_A12  (PIN_A11 + 1)
  static const uint8_t A12 = PIN_A12;
#endif
#if ANALOG_PINS_NUM > 13
  #define PIN_A13  (PIN_A12 + 1)
  static const uint8_t A13 = PIN_A13;
#endif
#if ANALOG_PINS_NUM > 14
  #define PIN_A14  (PIN_A13 + 1)
  static const uint8_t A14 = PIN_A14;
#endif
#if ANALOG_PINS_NUM > 15
  #define PIN_A15  (PIN_A14 + 1)
  static const uint8_t A15 = PIN_A15;
#endif
#if ANALOG_PINS_NUM > 16
  #define PIN_A16  (PIN_A15 + 1)
  static const uint8_t A16 = PIN_A16;
#endif
#if ANALOG_PINS_NUM > 17
  #define PIN_A17  (PIN_A16 + 1)
  static const uint8_t A17 = PIN_A17;
#endif
#if ANALOG_PINS_NUM > 18
  #define PIN_A18  (PIN_A17 + 1)
  static const uint8_t A18 = PIN_A18;
#endif
#if ANALOG_PINS_NUM > 19
  #define PIN_A19  (PIN_A18 + 1)
  static const uint8_t A19 = PIN_A19;
#endif
#if ANALOG_PINS_NUM > 20
  #define PIN_A20  (PIN_A19 + 1)
  static const uint8_t A20 = PIN_A20;
#endif
#if ANALOG_PINS_NUM > 21
  #define PIN_A21  (PIN_A20 + 1)
  static const uint8_t A21 = PIN_A21;
#endif
#if ANALOG_PINS_NUM > 22
  #define PIN_A22  (PIN_A21 + 1)
  static const uint8_t A22 = PIN_A22;
#endif
#if ANALOG_PINS_NUM > 23
  #define PIN_A23  (PIN_A22 + 1)
  static const uint8_t A23 = PIN_A23;
#endif
#if ANALOG_PINS_NUM > 24
  #define PIN_A24  (PIN_A23 + 1)
  static const uint8_t A24 = PIN_A24;
#endif
#if ANALOG_PINS_NUM > 25
  #define PIN_A25  (PIN_A24 + 1)
  static const uint8_t A25 = PIN_A25;
#endif
#if ANALOG_PINS_NUM > 26
  #define PIN_A26  (PIN_A25 + 1)
  static const uint8_t A26 = PIN_A26;
#endif
#if ANALOG_PINS_NUM > 27
  #define PIN_A27  (PIN_A26 + 1)
  static const uint8_t A27 = PIN_A27;
#endif
#if ANALOG_PINS_NUM > 28
  #define PIN_A28  (PIN_A27 + 1)
  static const uint8_t A28 = PIN_A28;
#endif
#if ANALOG_PINS_NUM > 29
  #define PIN_A29  (PIN_A28 + 1)
  static const uint8_t A29 = PIN_A29;
#endif
#if ANALOG_PINS_NUM > 30
  #define PIN_A30  (PIN_A29 + 1)
  static const uint8_t A30 = PIN_A20;
#endif
#if ANALOG_PINS_NUM > 31
  #define PIN_A31  (PIN_A30 + 1)
  static const uint8_t A31 = PIN_A31;
#endif
#define ANALOG_PINS_MAX   32
#else
#define ANALOG_PINS_MAX   0
#endif /* ANALOG_PINS_NUM > 0 */

#endif /* _ARDUINO_DIGITAL_PINS_H_ */
