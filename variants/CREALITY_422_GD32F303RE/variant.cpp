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

#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

const PinName digital_pins[] = {
  PORTA_0,
  PORTA_1,
  PORTA_2,
  PORTA_3,
  PORTA_4,
  PORTA_5,
  PORTA_6,
  PORTA_7,
  PORTA_8,
  PORTA_9,  // RXD
  PORTA_10, // TXD
  PORTA_11, // USB D-
  PORTA_12, // USB D+
  PORTA_13, // JTDI
  PORTA_14, // JTCK
  PORTA_15,
  PORTB_0,
  PORTB_1,
  PORTB_2,
  PORTB_3,  // JTDO
  PORTB_4,  // JTRST
  PORTB_5,
  PORTB_6,
  PORTB_7,
  PORTB_8,
  PORTB_9,
  PORTB_10,
  PORTB_11,  // LED
  PORTB_12,
  PORTB_13,
  PORTB_14,
  PORTB_15,
  PORTC_0,
  PORTC_1,
  PORTC_2,
  PORTC_3,
  PORTC_4,
  PORTC_5,
  PORTC_6,
  PORTC_7,
  PORTC_8,
  PORTC_9,
  PORTC_10,
  PORTC_11,
  PORTC_12,
  PORTC_13,
  PORTC_14,  // OSC32_1
  PORTC_15,  // OSC32_2
  PORTD_0,   // OSCIN
  PORTD_1,   // OSCOUT
  PORTD_2
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,  // A0,  PA0
  1,  // A1,  PA1
  2,  // A2,  PA2
  3,  // A3,  PA3
  4,  // A4,  PA4
  5,  // A5,  PA5
  6,  // A6,  PA6
  7,  // A7,  PA7
  16, // A8,  PB0
  17, // A9,  PB1
  32, // A10, PC0
  33, // A11, PC1
  34, // A12, PC2
  35, // A13, PC3
  36, // A14, PC4
  37  // A15, PC5
};

#ifdef __cplusplus
}
#endif