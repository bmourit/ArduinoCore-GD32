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

  Based on mbed-os/hal/include/hal/analogin_api.c
  Based on mbed-os/hal/include/hal/analogout_api.c
*/

#ifndef __ANALOG_H
#define __ANALOG_H

#include "gd32xxyy.h"

#include "PinNames.h"
#include "PeripheralPins.h"
#include "api/ArduinoAPI.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t isactive;
} analog_t;

uint8_t get_adc_channel(PinName pn);
uint8_t get_adc_index(uint32_t instance);
uint8_t get_dac_index(uint32_t instance);
void adc_clock_enable(uint32_t instance);

void set_dac_value(PinName pn, uint16_t value, uint8_t needs_init);
void dac_stop(PinName pn);
void set_pwm_value(pin_size_t pin, uint32_t value);
void set_pwm_value_with_base_period(pin_size_t pin, uint32_t base_period_us, uint32_t value);
void stop_pwm(pin_size_t pin);
uint16_t get_adc_value(PinName pn);

#ifdef __cplusplus
}
#endif

#endif /* __ANALOG_H */
