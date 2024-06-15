/**
 *  Copyright (c) 2024, BMourit ProjectHALGD
 *
 *  This HAL library is a distict, but derived from,
 *  the GigaDevice provided firmware files, so:
 *
 *  Copyright (c) 2020, GigaDevice Semiconductor Inc.
 *  
 *  Which in turn are clearly based on code from STMicroelectronics. We'll attempt
 *  to right the wrong of GigaDevice with:
 *
 *  Copyright (c) 2017, STMicroelectronics. All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without modification, 
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this 
 *      list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, 
 *      this list of conditions and the following disclaimer in the documentation 
 *      and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the names of its contributors 
 *      may be used to endorse or promote products derived from this software without 
 *      specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 *  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 *  OF SUCH DAMAGE.
 */

#ifndef GPIO_EXTRA_H
#define GPIO_EXTRA_H

#include "gd32xxyy_spl_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

__STATIC_INLINE void gpio_digital_toggle(uint32_t gpiox, uint32_t pins)
{
	GPIO_OCTL(gpiox) = ((GPIO_OCTL(gpiox)) ^ (pins & 0x0000FFFFU));
}

__STATIC_INLINE void gpio_pin_set_output(uint32_t gpiox, uint32_t pins)
{
	GPIO_BOP(gpiox) = (pins & 0x0000FFFFU);
}

__STATIC_INLINE void gpio_pin_reset_output(uint32_t gpiox, uint32_t pins)
{
	GPIO_BC(gpiox) = (pins & 0x0000FFFFU);
}

static inline void gpio_digital_write(uint32_t gpiox, uint32_t pin, uint32_t value)
{
	if (value) {
		gpio_pin_set_output(gpiox, pin);
	} else {
		gpio_pin_reset_output(gpiox, pin);
	}
}

#ifdef __cplusplus
}
#endif

#endif /* GPIO_EXTRA_H */
