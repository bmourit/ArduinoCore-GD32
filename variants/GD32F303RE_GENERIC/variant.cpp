/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#if defined(ARDUINO_GENERIC_F303RETX)
#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/* digital pins for pinmap list */
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
    PORTA_9,
    PORTA_10,
    PORTA_11,
    PORTA_12,
    PORTA_13,
    PORTA_14,
    PORTA_15,
    PORTB_0,
    PORTB_1,
    PORTB_2,
    PORTB_3,
    PORTB_4,
    PORTB_5,
    PORTB_6,
    PORTB_7,
    PORTB_8,
    PORTB_9,
    PORTB_10,
    PORTB_11,
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
    PORTC_14,
    PORTC_15,
    PORTD_0,
    PORTD_1,
    PORTD_2
};

/* analog pins for pinmap list */
const uint32_t analog_pins[] = {
    0,//PA0, //A0  //Ardunio A0
    1,//PA1, //A1  //Ardunio A1
    2,//PA2, //A2  //Ardunio A2
    3,//PA3, //A3  //Ardunio A3
    4,//PA4, //A4  //Ardunio A4
    5,//PA5, //A5  //Ardunio A5
    6,//PA6, //A6
    7,//PA7, //A7
    16,//PB0, //A8
    17,//PB1, //A9
    32,//PC0, //A10
    33,//PC1, //A11
    34,//PC2, //A12
    35,//PC3, //A13
    36,//PC4, //A14
    37//PC5, //A15
};
#ifdef __cplusplus
}
#endif
#endif
