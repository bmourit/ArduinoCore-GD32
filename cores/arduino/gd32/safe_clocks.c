/**
 *  Copyright (c) 2024, BMourit
 *	
 * 	Safe Clocks
 *  Provides functions for proper and safe startup of the system clocks
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

#include <stddef.h>

#include "backup_domain.h"
#include "safe_clocks.h"
//#include "gd32_def.h"

static inline void CLOCKS_DELAY(uint32_t delay) {
	uint32_t cycles_per_ms = (SystemCoreClock / 1000U);
	volatile uint32_t i;
	for (i = 0; i < delay * cycles_per_ms; i++) {
	}
}

/**
 * Firmware code provided by GigaDevices has several issues.
 * One being the startup code not configuring a wait state
 * value for the flash memory, which is required to be able to
 * read from flash.
 * This function sets the wait state value, which can be
 * one of the following:
 * 		WS_WSCNT_0: no wait
 * 		WS_WSCNT_1: 1 wait cycle
 * 		WS_WSCNT_2:	2 wait cycles
 * Returns an SC_error_t
 */
SC_error_t SC_set_flash_wait_state(uint32_t wait_state_value)
{
	uint32_t reg = 0U;

	/* we cannot read from flash without the proper wait state configured */
	reg = FMC_WS;
	reg &= ~FMC_WS_WSCNT;
	reg |= wait_state_value;
	FMC_WS = reg;

	/* read the value to make sure it is set */
	reg = FMC_WS;
	reg &= FMC_WS_WSCNT;
	if (reg != wait_state_value) {
		return SC_ERROR;
	}

	return SC_OK;
}

/**
 * Another issue with the vendor provided code is
 * the lack of safety checks. It blindly sets the dividers
 * for AHB/APB1/APB2 without considering any possibiliy of
 * falling out of spec during startup.
 * 
 * This function sets the dividers to the max possible during
 * startup to prevent any possibility of falling out of spec.
 * It then sets the dividers to the desired value once it is
 * safe to do so.
 * 
 * Finally, it updates the CMSIS global SystemCoreClock parameter
 * to the proper value.
 */
SC_error_t SC_Clock_Params(SC_clock_params_t *clock_params, uint32_t wait_state_value)
{
	uint32_t reg = 0U;
	volatile uint32_t timeout = 0U;

	if (clock_params == NULL) {
		return SC_ERROR;
	}

#if defined(FMC_WS_WSCNT)
	/* faster system clocks require a longer wait state */
	reg = FMC_WS;
	if ((reg & FMC_WS_WSCNT) < wait_state_value) {
		if (SC_set_flash_wait_state(wait_state_value) != SC_OK) {
			return SC_ERROR;
		}
	}
#endif

	if (((clock_params->clock) & RCU_CLK_AHB) == RCU_CLK_AHB) {
		/* temporarily set APB1 divider to max */
		if (((clock_params->clock) & RCU_CLK_APB1) == RCU_CLK_APB1) {
			reg = RCU_CFG0;
			reg &= ~RCU_CFG0_APB1PSC;
			reg |= RCU_APB1_CKAHB_DIV16;
			RCU_CFG0 = reg;
		}
		/* temporarily set APB2 divider to max */
		if (((clock_params->clock) & RCU_CLK_APB2) == RCU_CLK_APB2) {
			reg = RCU_CFG0;
			reg &= ~RCU_CFG0_APB2PSC;
			reg |= RCU_APB2_CKAHB_DIV16;
			RCU_CFG0 = reg;
		}
		/* set the real AHB divider */
		reg = RCU_CFG0;
		reg &= ~RCU_CFG0_AHBPSC;
		reg |= clock_params->ahbclk_div;
		RCU_CFG0 = reg;
	}
	/* check desired system source clock is ready */
	if (((clock_params->clock) & RCU_CLK_SYS) == RCU_CLK_SYS) {
		if (clock_params->system_source == RCU_CKSYSSRC_HXTAL) {
			if (rcu_flag_get(RCU_FLAG_HXTALSTB) == RESET) {
				return SC_ERROR;
			}
		} else if (clock_params->system_source == RCU_CKSYSSRC_PLL) {
			if (rcu_flag_get(RCU_FLAG_PLLSTB) == RESET) {
				return SC_ERROR;
			}
		} else {
			if (rcu_flag_get(RCU_FLAG_IRC8MSTB) == RESET) {
				return SC_ERROR;
			}
		}
		/* set the system clock source */
		reg = RCU_CFG0;
		reg &= ~RCU_CFG0_SCS;
		reg |= clock_params->system_source;
		RCU_CFG0 = reg;

		/* wait for current source to be the select source */
		do {
			timeout++;
		} while (((RCU_CFG0 & RCU_CFG0_SCSS) != clock_params->system_source) && (timeout != HXTAL_STARTUP_TIMEOUT));
		if (timeout >= HXTAL_STARTUP_TIMEOUT) {
			return SC_TIMEOUT;
		}
	}

#if defined(FMC_WS_WSCNT)
	/* lower system clocks require a lower wait state */
	reg = FMC_WS;
	if ((reg & FMC_WS_WSCNT) > wait_state_value) {
		if (SC_set_flash_wait_state(wait_state_value) != SC_OK) {
			return SC_ERROR;
		}
	}
#endif

	/* set the real APB1 divider */
	if (((clock_params->clock) & RCU_CLK_APB1) == RCU_CLK_APB1) {
		reg = RCU_CFG0;
		reg &= ~RCU_CFG0_APB1PSC;
		reg |= clock_params->apb1clk_div;
		RCU_CFG0 = reg;
	}
	/* set the real APB2 divider */
	if (((clock_params->clock) & RCU_CLK_APB2) == RCU_CLK_APB2) {
		reg = RCU_CFG0;
		reg &= ~RCU_CFG0_APB2PSC;
		reg |= clock_params->apb2clk_div;
		RCU_CFG0 = reg;
	}

	/* update the SystemCoreClock CMSIS global variable */
	SystemCoreClock = SC_get_system_clock_frequency() >> AHBPrescaler[((RCU_CFG0 & RCU_CFG0_AHBPSC) >> 4)];

	return SC_OK;
}

uint32_t SC_get_system_clock_frequency(void)
{
	const uint8_t pll_multipliers[32] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 29, 29, 30, 31, 32};
	const uint8_t clock_prescalers[2] = {1, 2};

	uint32_t reg = 0U, clkpsc = 0U, pllclk = 0U, pllmf = 0U, pllmf_tmp = 0U;
	uint32_t sysclk_freq = 0U;

	reg = RCU_CFG0;
	switch (reg & RCU_CFG0_SCSS) {
		case RCU_SCSS_HXTAL: {
			sysclk_freq = HXTAL_VALUE;
			break;
		}
		case RCU_SCSS_PLL: {
			pllmf_tmp = ((((reg & PLLMF_4) >> 27) << 4) | ((reg & RCU_CFG0_PLLMF) >> 18));
			pllmf = pll_multipliers[(uint32_t)pllmf_tmp];
			if ((reg & RCU_CFG0_PLLSEL) != RCU_PLLSRC_IRC8M_DIV2) {
				clkpsc = clock_prescalers[(uint32_t)(RCU_CFG0 & RCU_CFG0_PREDV0) >> 17];
				pllclk = (uint32_t)((HXTAL_VALUE * pllmf) / clkpsc);
			} else {
				/* IRC8M / 2 is system clock */
				pllclk = (uint32_t)((IRC8M_VALUE >> 1) * pllmf);
			}
			sysclk_freq = pllclk;
			break;
		}
		case RCU_SCSS_IRC8M:
		default: {
			sysclk_freq = IRC8M_VALUE;
			break;
		}
	}

	return sysclk_freq;
}

SC_error_t SC_Osc_Params(SC_oscillator_params_t *osc_params)
{
	uint32_t timeout = 0U;
	uint32_t reg = 0U;

	if (osc_params == NULL) {
		return SC_ERROR;
	}

	if (((osc_params->osc) & RCU_OSC_HXTAL) == RCU_OSC_HXTAL) {
		/* if HXTAL is sytem clock it can never be turned off */
		if (((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_HXTAL) || (((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLL) && ((RCU_CFG0 & RCU_CFG0_PLLSEL) == RCU_PLLSRC_HXTAL_IRC48M))) {
			if ((rcu_flag_get(RCU_FLAG_HXTALSTB) != RESET) && (osc_params->HXTAL_state == RCU_HXTAL_OFF)) {
				return SC_ERROR;
			}
		} else {
			do {
				if ((osc_params->HXTAL_state) == RCU_HXTAL_ON) {
					reg = RCU_CTL;
					reg &= ~RCU_CTL_HXTALEN;
					reg |= RCU_CTL_HXTALEN;
					RCU_CTL = reg;
					CLOCKS_DELAY(10000);
				} else if ((osc_params->HXTAL_state) == RCU_HXTAL_OFF) {
					reg = RCU_CTL;
					reg &= ~RCU_CTL_HXTALBPS;
					RCU_CTL = reg;
					CLOCKS_DELAY(5000);
					reg = RCU_CTL;
					reg &= ~RCU_CTL_HXTALEN;
					RCU_CTL = reg;
				} else if ((osc_params->HXTAL_state) == RCU_HXTAL_BYPASS) {
					reg = RCU_CTL;
					reg &= ~RCU_CTL_HXTALBPS;
					reg |= RCU_CTL_HXTALBPS;
					RCU_CTL = reg;
					CLOCKS_DELAY(5000);
					reg = RCU_CTL;
					reg &= ~RCU_CTL_HXTALEN;
					reg |= RCU_CTL_HXTALEN;
					RCU_CTL = reg;
					CLOCKS_DELAY(10000);
				} else {
					reg = RCU_CTL;
					reg &= ~RCU_CTL_HXTALBPS;
					RCU_CTL = reg;
					CLOCKS_DELAY(5000);
					reg = RCU_CTL;
					reg &= ~RCU_CTL_HXTALEN;
					reg |= RCU_CTL_HXTALEN;
					RCU_CTL = reg;
				}
			} while (0U);
			if (osc_params->HXTAL_state != RCU_HXTAL_OFF) {
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_HXTALSTB) == RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
			} else {
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_HXTALSTB) != RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
			}
		}
	}
	if (((osc_params->osc) & RCU_OSC_IRC8M) == RCU_OSC_IRC8M) {
		if (((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_IRC8M) || (((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLL) && ((RCU_CFG0 & RCU_CFG0_PLLSEL) == RCU_PLLSRC_IRC8M_DIV2))) {
			if ((rcu_flag_get(RCU_FLAG_IRC8MSTB) != RESET) && (osc_params->IRC8M_state != RCU_IRC8M_ON)) {
				return SC_ERROR;
			} else {
				reg = RCU_CTL;
				reg &= ~RCU_CTL_IRC8MADJ;
				RCU_CTL = (reg | ((uint32_t)(osc_params->IRC8M_calibration) << 3U));
				CLOCKS_DELAY(5000);
			}
		} else {
			if (osc_params->IRC8M_state != RCU_IRC8M_OFF) {
				reg = RCU_CTL;
				reg &= ~RCU_CTL_IRC8MEN;
				RCU_CTL = (reg | RCU_CTL_IRC8MEN);
				CLOCKS_DELAY(2000);
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_IRC8MSTB) == RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
				reg = RCU_CTL;
				reg &= ~RCU_CTL_IRC8MADJ;
				RCU_CTL = (reg | ((uint32_t)(osc_params->IRC8M_calibration) << 3U));
				CLOCKS_DELAY(2000);
			} else {
				reg = RCU_CTL;
				reg &= ~RCU_CTL_IRC8MEN;
				RCU_CTL = reg;
				CLOCKS_DELAY(2000);
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_IRC8MSTB) != RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
			}
		}
	}
	if (((osc_params->osc) & RCU_OSC_IRC40K) == RCU_OSC_IRC40K) {
		if (osc_params->IRC40K_state != RCU_IRC40K_OFF) {
			reg = RCU_RSTSCK;
			reg &= ~RCU_RSTSCK_IRC40KEN;
			reg |= RCU_RSTSCK_IRC40KEN;
			RCU_RSTSCK = reg;
			CLOCKS_DELAY(2000);
			do {
				timeout++;
			} while ((rcu_flag_get(RCU_FLAG_IRC40KSTB) == RESET) && (timeout != HXTAL_STARTUP_TIMEOUT + 1));
			if (timeout >= HXTAL_STARTUP_TIMEOUT + 1) {
				return SC_TIMEOUT;
			}
		} else {
			reg = RCU_RSTSCK;
			reg &= ~RCU_RSTSCK_IRC40KEN;
			RCU_RSTSCK = reg;
			CLOCKS_DELAY(2000);
			do {
				timeout++;
			} while ((rcu_flag_get(RCU_FLAG_IRC40KSTB) != RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
			if (timeout >= HXTAL_STARTUP_TIMEOUT) {
				return SC_TIMEOUT;
			}
		}
	}

	if (((osc_params->osc) & RCU_OSC_LXTAL) == RCU_OSC_LXTAL) {
		FlagStatus pmu_clk_changed = RESET;
		if ((RCU_APB1EN & (RCU_APB1EN_PMUEN)) == RESET) {
			do {
				__IO uint32_t temp;
				reg = RCU_APB1EN;
				reg &= ~RCU_APB1EN_PMUEN;
				reg |= RCU_APB1EN_PMUEN;
				RCU_APB1EN = reg;
				/* read for delay */
				temp = (RCU_APB1EN & (RCU_APB1EN_PMUEN));
			} while (0U);
			pmu_clk_changed = SET;
		}
		if (((PMU_CTL & PMU_CTL_BKPWEN) >> 8) == 0U) {
			reg = PMU_CTL;
			reg &= ~PMU_CTL_BKPWEN;
			reg |= PMU_CTL_BKPWEN;
			PMU_CTL = reg;

			do {
				timeout++;
			} while ((((PMU_CTL & PMU_CTL_BKPWEN) >> 8) == 0U) && (timeout != HXTAL_STARTUP_TIMEOUT));
			if (timeout >= HXTAL_STARTUP_TIMEOUT) {
				return SC_TIMEOUT;
			}
		}
		do {
			if (osc_params->LXTAL_state == RCU_LXTAL_ON) {
				reg = RCU_BDCTL;
				reg &= ~RCU_BDCTL_LXTALEN;
				reg |= RCU_BDCTL_LXTALEN;
				RCU_BDCTL = reg;
			} else if (osc_params->LXTAL_state == RCU_LXTAL_OFF) {
				/* disable LXTAL */
				reg = RCU_BDCTL;
				reg &= ~RCU_BDCTL_LXTALEN;
				RCU_BDCTL = reg;
				/* disable bypass */
				reg = RCU_BDCTL;
				reg &= ~RCU_BDCTL_LXTALBPS;
				RCU_BDCTL = reg; 
			} else if (osc_params->LXTAL_state == RCU_LXTAL_BYPASS) {
				/* enable bypass */
				reg = RCU_BDCTL;
				reg &= ~RCU_BDCTL_LXTALBPS;
				reg |= RCU_BDCTL_LXTALBPS;
				RCU_BDCTL = reg; 
				/* enable LXTAL */
				reg = RCU_BDCTL;
				reg &= ~RCU_BDCTL_LXTALEN;
				reg |= RCU_BDCTL_LXTALEN;
				RCU_BDCTL = reg;
			} else {
				/* disable LXTAL */
				reg = RCU_BDCTL;
				reg &= ~RCU_BDCTL_LXTALEN;
				RCU_BDCTL = reg;
				/* disable bypass */
				reg = RCU_BDCTL;
				reg &= ~RCU_BDCTL_LXTALBPS;
				RCU_BDCTL = reg; 
			}
		} while (0U);

		if (osc_params->LXTAL_state != RCU_LXTAL_OFF) {
			do {
				timeout++;
			} while ((rcu_flag_get(RCU_FLAG_LXTALSTB) == RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
			if (timeout >= HXTAL_STARTUP_TIMEOUT) {
				return SC_TIMEOUT;
			}
		} else {
			do {
				timeout++;
			} while ((rcu_flag_get(RCU_FLAG_LXTALSTB) != RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
			if (timeout >= HXTAL_STARTUP_TIMEOUT) {
				return SC_TIMEOUT;
			}
		}
		if (pmu_clk_changed == SET) {
			reg = RCU_APB1EN;
			reg &= ~RCU_APB1EN_PMUEN;
			RCU_APB1EN = reg;
		}

	}

	if ((osc_params->pll_params.pll_status) != RCU_PLL_NONE) {
		if ((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_PLL) {
			if ((osc_params->pll_params.pll_status) == RCU_PLL_ON) {
				reg = RCU_CTL;
				reg &= ~RCU_CTL_PLLEN;
				RCU_CTL = reg;
				CLOCKS_DELAY(4000);
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_PLLSTB) != RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
				if (osc_params->pll_params.pll_source_clock == RCU_PLLSRC_HXTAL_IRC48M) {
					reg = RCU_CFG0;
					reg &= ~RCU_CFG0_PREDV0;
					if (osc_params->HXTAL_prediv == RCU_HXTAL_PREDIV2) {
						reg |= RCU_CFG0_PREDV0;
					}
					RCU_CFG0 = reg;
					CLOCKS_DELAY(4000);
				}

				/* set pll source and pllmf */
				reg = RCU_CFG0;
				reg &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
				reg |= (osc_params->pll_params.pll_source_clock | osc_params->pll_params.pll_multiplier);
				RCU_CFG0 = reg;
				CLOCKS_DELAY(5000);

				/* enable pll */
				reg = RCU_CTL;
				reg &= ~RCU_CTL_PLLEN;
				RCU_CTL = (reg | RCU_CTL_PLLEN);
				CLOCKS_DELAY(2000);
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_PLLSTB) == RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
			} else {
				reg = RCU_CTL;
				reg &= ~RCU_CTL_PLLEN;
				RCU_CTL = reg;
				CLOCKS_DELAY(4000);
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_PLLSTB) != RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
			}
		} else {
			if ((osc_params->pll_params.pll_status) == RCU_PLL_OFF) {
				return SC_ERROR;
			} else {
				reg = RCU_CFG0;
				if (((reg & RCU_CFG0_PLLSEL) != osc_params->pll_params.pll_source_clock) ||
						((reg & RCU_CFG0_PLLMF) != osc_params->pll_params.pll_multiplier)) {
					return SC_ERROR;
				}
			}
		}
	}

	return SC_OK;
}

SC_error_t SC_Periph_Params(SC_peripheral_params_t *periph_params)
{
	uint32_t reg = 0U;
	uint32_t timeout = 0U;

	if ((((periph_params->pclock) & RCU_PERIPHCLK_RTC) == RCU_PERIPHCLK_RTC)) {
		FlagStatus pmu_clk_changed = RESET;

		if ((RCU_APB1EN & (RCU_APB1EN_PMUEN)) == RESET) {
			do {
				__IO uint32_t temp;
				reg = RCU_APB1EN;
				reg &= ~RCU_APB1EN_PMUEN;
				reg |= RCU_APB1EN_PMUEN;
				RCU_APB1EN = reg;
				/* read for delay */
				temp = (RCU_APB1EN & (RCU_APB1EN_PMUEN));
			} while (0U);
			pmu_clk_changed = SET;
		}

		if (((PMU_CTL & PMU_CTL_BKPWEN) >> 8) == 0U) {
			reg = PMU_CTL;
			reg &= ~PMU_CTL_BKPWEN;
			reg |= PMU_CTL_BKPWEN;
			PMU_CTL = reg;

			do {
				timeout++;
			} while ((((PMU_CTL & PMU_CTL_BKPWEN) >> 8) == 0U) && (timeout != HXTAL_STARTUP_TIMEOUT));
			if (timeout >= HXTAL_STARTUP_TIMEOUT) {
				return SC_TIMEOUT;
			}
		}
		reg = (RCU_BDCTL & RCU_BDCTL_RTCSRC);
		if ((reg != 0x00000000U) && (reg != (periph_params->rtc_clk & RCU_BDCTL_RTCSRC))) {
			reg = (RCU_BDCTL & ~(RCU_BDCTL_RTCSRC));
			rcu_bkp_reset_enable();
			rcu_bkp_reset_disable();
			RCU_BDCTL = reg;

			if ((reg & RCU_BDCTL_LXTALEN) == 1U) {
				do {
					timeout++;
				} while ((rcu_flag_get(RCU_FLAG_LXTALSTB) == RESET) && (timeout != HXTAL_STARTUP_TIMEOUT));
				if (timeout >= HXTAL_STARTUP_TIMEOUT) {
					return SC_TIMEOUT;
				}
			}
		}
		reg = RCU_BDCTL;
		reg &= ~RCU_BDCTL_RTCSRC;
		reg |= periph_params->rtc_clk;
		RCU_BDCTL = RCU_BDCTL;

		if (pmu_clk_changed == SET) {
			reg = PMU_CTL;
			reg &= ~PMU_CTL_BKPWEN;
			PMU_CTL = reg;
		}
	}

	if (((periph_params->pclock) & RCU_PERIPHCLK_ADC) == RCU_PERIPHCLK_ADC) {
		reg = RCU_CFG0;
		reg &= ~(RCU_CFG0_ADCPSC_2 | RCU_CFG0_ADCPSC);
		reg |= (periph_params->adc_clk << 14);
		RCU_CFG0 = reg;
	}

	if (((periph_params->pclock) & RCU_PERIPHCLK_USB) == RCU_PERIPHCLK_USB) {
		//TODO
	}

	return SC_OK;
}


void clockEnable(clock_source_t clock_source)
{
  SC_oscillator_params_t osc_params = {0};
  osc_params.pll_params.pll_status = RCU_PLL_NONE;

  backup_domain_enable();

  switch(clock_source) {
    case SOURCE_IRC40K:
      if (rcu_flag_get(RCU_FLAG_IRC40KSTB) == RESET) {
        osc_params.osc = RCU_OSC_IRC40K;
        osc_params.IRC40K_state = RCU_IRC40K_ON;
      }
      break;
    case SOURCE_IRC8M:
      if (rcu_flag_get(RCU_FLAG_IRC8MSTB) == RESET) {
        osc_params.osc = RCU_OSC_IRC8M;
        osc_params.IRC8M_state = RCU_IRC8M_ON;
        osc_params.IRC8M_calibration = 16U;
      }
      break;
  case SOURCE_LXTAL:
      if (rcu_flag_get(RCU_FLAG_LXTALSTB) == RESET) {
#ifdef USE_LXTAL_DRIVE_CAP
        rcu_lxtal_drive_capability_config(RCU_LXTAL_LOWDRI);
#endif
        osc_params.osc = RCU_OSC_LXTAL;
        osc_params.LXTAL_state = RCU_LXTAL_ON;
      }
      break;
  case SOURCE_HXTAL:
      if (rcu_flag_get(RCU_FLAG_HXTALSTB) == RESET) {
        osc_params.osc = RCU_OSC_HXTAL;
        osc_params.HXTAL_state = RCU_HXTAL_ON;
      }
      break;
  default:
      break;
  }

  if (osc_params.osc != RCU_OSC_NONE) {
    if (SC_Osc_Params(&osc_params) != SC_OK) {
      Error_Handler();
    }
  }
}
