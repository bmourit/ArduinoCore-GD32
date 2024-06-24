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

#ifndef GD32F30X_SAFE_CLOCKS_H
#define GD32F30X_SAFE_CLOCKS_H

#include "gd32f30x_rcu.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RCU_HXTAL_OFF		0x00000000U
#define RCU_HXTAL_ON		RCU_CTL_HXTALEN
#define RCU_HXTAL_BYPASS	((uint32_t)(RCU_CTL_HXTALEN | RCU_CTL_HXTALBPS))

#define RCU_PLL_NONE		0x00000000U
#define RCU_PLL_OFF			0x00000001U
#define RCU_PLL_ON			0x00000002U

#define RCU_IRC4M_OFF		0x00000000U
#define RCU_IRC8M_ON		RCU_CTL_IRC8MEN

#define RCU_IRC40K_OFF		0x00000000U
#define RCU_IRC40K_ON		RCU_RSTSCK_IRC40KEN

#define RCU_HXTAL_PREDIV1	0U;
#define RCU_HXTAL_PREDIV2	1U;

typedef struct sc_pll_params_struct {
	uint32_t pll_status;				/* pll_status: no_config, on, or off */
	uint32_t pll_source_clock;			/* either IRC8M / 2, or HXTAL */
	uint32_t pll_multiplier;			/* the multiplier of the pll clock */
} SC_pll_params_t;

typedef struct sc_clock_params_struct {
	uint32_t clock;						/* sysclk, ahbclk, apb1clk, or apb2clk */
	uint32_t system_source;				/* one of the RCU_CKSYSSRC Source values */
	uint32_t ahbclk_div;
	uint32_t apb1clk_div;
	uint32_t apb2clk_div;
} SC_clock_params_t;

typedef struct sc_oscillator_params_struct {
	uint32_t osc;
	uint32_t HXTAL_state;
	uint32_t HXTAL_prediv;
	uint32_t LXTAL_state;
	uint32_t IRC8M_state;
	uint32_t IRC8M_calibration;
	uint32_t IRC40K_state;
	SC_pll_params_t pll_params;
} SC_oscillator_params_t;

typedef struct sc_peripheral_params_struct {
	uint32_t pclock;
	uint32_t rtc_clk;
	uint32_t adc_clk;
	uint32_t i2s1_clk;
	uint32_t i2s2_clk;
	uint32_t usb_clk;
} SC_peripheral_params_t;

typedef enum {
   RCU_CLK_SYS = 0x00000001U,			/* system clock */
   RCU_CLK_AHB = 0x00000002U,			/* AHB clock */
   RCU_CLK_APB1 = 0x00000004U,			/* APB1 clock */
   RCU_CLK_APB2 = 0x00000008U			/* APB2 clock */
} SC_clock_t;

typedef enum {
	RCU_OSC_NONE = 0x00000001U,
	RCU_OSC_HXTAL = 0x00000001U,
	RCU_OSC_IRC8M = 0x00000002U,
	RCU_OSC_LXTAL = 0x00000004U,
	RCU_OSC_IRC40K = 0x00000008U
} SC_osc_t;

typedef enum {
	RCU_PERIPHCLK_RTC - 0x00000001U,
	RCU_PEERIPHCLK_ADC = 0x00000002U,
	RCU_PEERIPHCLK_I2S0 = 0x00000004U,
	RCU_PEERIPHCLK_I2S1 = 0x00000008U,
	RCU_PEERIPHCLK_USB = 0x00000010U
} SC_periph_t;

/* safe clock errors */
typedef enum {
  SC_OK = 0U,
  SC_ERROR = 1U,
  SC_TIMEOUT = 2U
} SC_error_t;

#define IRC8M_CALIBRATION_DEFAULT		16U

SC_error_t SC_Osc_Params(SC_oscillator_params_t *osc_params);
/* safe clock flash wait state */
SC_error_t SC_set_flash_wait_state(uint32_t wait_state_value);
/* safe clock params */
SC_error_t SC_Clock_Params(SC_clock_params_t *clock_params, uint32_t flash_wait_value);
SC_error_t SC_Periph_Params(SC_peripheral_params_t *periph_params);
/* get the clock frequency to update the SystemCoreClock global paramater */
uint32_t SC_get_system_clock_frequency(void);

#ifdef __cplusplus
}
#endif

#endif /* GD32F30X_BITS_RCU_H */
