/*!
	\file  system_gd32f30x.c
	\brief CMSIS Cortex-M4 Device Peripheral Access Layer Source File for
			 GD32F30x Device Series
*/

/* Copyright (c) 2012 ARM LIMITED

	 All rights reserved.
	 Redistribution and use in source and binary forms, with or without
	 modification, are permitted provided that the following conditions are met:
	 - Redistributions of source code must retain the above copyright
	 notice, this list of conditions and the following disclaimer.
	 - Redistributions in binary form must reproduce the above copyright
	 notice, this list of conditions and the following disclaimer in the
	 documentation and/or other materials provided with the distribution.
	 - Neither the name of ARM nor the names of its contributors may be used
	 to endorse or promote products derived from this software without
	 specific prior written permission.
	 *
	 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	 ARE DISCLAIMED.w IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
	 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
	 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
	 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
	 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
	 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	 POSSIBILITY OF SUCH DAMAGE.
	 ---------------------------------------------------------------------------*/
/* This file refers to the CMSIS standard, some adjustments are made according to GigaDevice chips */

#include "gd32f30x.h"

/* system frequency define */
#define __IRC8M         (IRC8M_VALUE)            /* internal 8 MHz RC oscillator frequency */
#define __HXTAL         (HXTAL_VALUE)            /* high speed crystal oscillator frequency */
#define __SYS_OSC_CLK   (__IRC8M)                /* main oscillator frequency */

#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET   0x00007000UL  /* Vector Table base offset field must be a multiple of 0x200 */
#endif

#define SEL_IRC8M   0x00U
#define SEL_HXTAL   0x01U
#define SEL_PLL     0x02U

/**
 * According to the GD32F30x manual, on power up
 * the system will reset to default values
 * Relevant here is the following:
 *    CK_SYS is set to use IRC8M and by default
 *    it multiplies by 2. This gives us an initial 
 *    clock value of 16000000. This is the correct
 *    value to use here.
 * 
 * With this value, there is no reason we need
 * to up the prescaler value of the AHB in the 
 * wait routine. The SystemCoreClock value should
 * reflect the actual clock, not the eventual final
 * clock. Setting this value to eg: 120000000 here
 * may lead to timing issues and is generally a
 * bad idea.
 */
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescaler[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};

static inline void CLOCKS_DELAY(uint32_t delay) {
    volatile uint32_t count;
    while (delay > 0) {
        // Simple delay loop
        for (count = 0; count < 1000; count++) {
            __asm__ volatile ("nop");
        }
        delay--;
    }
}

/*!
 * \brief Initialize the system clock and vector table
 */
void SystemInit(void)
{
	// Reset the RCU clock configuration to the default reset state
	RCU_CTL |= 0x00000001U;
	//CLOCKS_DELAY(2000UL);

	// Reset SCS, SCSS, AHBPSC, APBPSC, APB1PSC, APB2PSC, ADCPSC, ADCPSC_2, and CKOUT0SEL
	//RCU_CFG0 &= 0xE8FF0000UL;
	RCU_CFG0 &= 0xF8FF0000U;

#if defined(GD32F30X_HD) || defined(GD32F30X_XD)
	// Reset HXTALEN, CKMEN, and PLLEN bits
	//RCU_CTL &= ~(RCU_CTL_PLLEN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
	RCU_CTL &= 0xFEF6FFFFU;
	RCU_CTL &= 0xFFFBFFFFU;
	RCU_CFG0 &= 0xFF80FFFFU;
#else
	// Reset HXTALEN, CKMEN, PLLEN, PLL1EN, and PLL2EN bits
	RCU_CTL &= ~(RCU_CTL_PLLEN | RCU_CTL_PLL1EN | RCU_CTL_PLL2EN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
#endif

	// Reset HXTALBPS bit
	//RCU_CTL &= ~(RCU_CTL_HXTALBPS);

	// Reset PLLSEL, PREDV0, PLLMF, USBDPSC, PLLMF_4, PLLMF_5, and USBDPSC_2 bits
	//RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0 | RCU_CFG0_PLLMF | RCU_CFG0_USBDPSC | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5 | RCU_CFG0_USBDPSC_2);

#if defined(GD32F30X_HD) || defined(GD32F30X_XD)
	// Disable all interrupts
	RCU_INT = 0x009F0000U;
#else
	// Reset PLL1EN and PLL2EN bits
	RCU_CTL &= ~(RCU_CTL_PLL1EN | RCU_CTL_PLL2EN);
	// Disable all interrupts
	RCU_INT = 0x00FF0000U;
#endif

	// Set the vector table offset
#ifdef VECT_TAB_SRAM
	nvic_vector_table_set(NVIC_VECTTAB_RAM, VECT_TAB_OFFSET);
#else
	nvic_vector_table_set(NVIC_VECTTAB_FLASH, VECT_TAB_OFFSET);
#endif
}

/*!
	\brief      configure the system clock to 120M by PLL which selects HXTAL(8M) as its clock source
	\param[in]  none
	\param[out] none
	\retval     none
*/
static void system_clock_120m_hxtal(void)
{
	uint32_t timeout = 0U;
	uint32_t stab_flag = 0U;

	/* enable HXTAL */
	RCU_CTL |= RCU_CTL_HXTALEN;

	/* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
	do {
		timeout++;
		stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
	} while ((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

	/* if fail */
	if (0U == (RCU_CTL & RCU_CTL_HXTALSTB)) {
		while (1) {
		}
	}

	RCU_APB1EN |= RCU_APB1EN_PMUEN;
	PMU_CTL |= PMU_CTL_LDOVS;

	/* HXTAL is stable */
	/* AHB = SYSCLK */
	RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
	/* APB2 = AHB/1 */
	RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
	/* APB1 = AHB/2 */
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

#if (defined(GD32F30X_HD) || defined(GD32F30X_XD))
	/* select HXTAL / 2 as clock source */
	RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
	RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_CFG0_PREDV0);

	/* CK_PLL = (CK_HXTAL / 2) * 30 = 120 MHz */
	RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
	RCU_CFG0 |= RCU_PLL_MUL30;

#elif defined(GD32F30X_CL)
	/* CK_PLL = (CK_PREDIV0) * 30 = 120 MHz */ 
	RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
	RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_PLL_MUL30);

	/* CK_PREDIV0 = (CK_HXTAL) / 5 * 8 / 10 = 4 MHz */ 
	RCU_CFG1 &= ~(RCU_CFG1_PLLPRESEL | RCU_CFG1_PREDV0SEL | RCU_CFG1_PLL1MF | RCU_CFG1_PREDV1 | RCU_CFG1_PREDV0);
	RCU_CFG1 |= (RCU_PLLPRESRC_HXTAL | RCU_PREDV0SRC_CKPLL1 | RCU_PLL1_MUL8 | RCU_PREDV1_DIV5 | RCU_PREDV0_DIV10);

	/* enable PLL1 */
	RCU_CTL |= RCU_CTL_PLL1EN;

	/* wait till PLL1 is ready */
	while ((RCU_CTL & RCU_CTL_PLL1STB) == 0U) {
	}
#endif /* GD32F30X_HD and GD32F30X_XD */

	/* enable PLL */
	RCU_CTL |= RCU_CTL_PLLEN;

	/* wait until PLL is stable */
	while (0U == (RCU_CTL & RCU_CTL_PLLSTB)) {
	}

	/* enable the high-drive to extend the clock frequency to 120 MHz */
	PMU_CTL |= PMU_CTL_HDEN;
	while (0U == (PMU_CS & PMU_CS_HDRF)) {
	}

	/* select the high-drive mode */
	PMU_CTL |= PMU_CTL_HDS;
	while (0U == (PMU_CS & PMU_CS_HDSRF)) {
	}

	/* select PLL as system clock */
	RCU_CFG0 &= ~RCU_CFG0_SCS;
	RCU_CFG0 |= RCU_CKSYSSRC_PLL;

	/* wait until PLL is selected as system clock */
	while (0U == (RCU_CFG0 & RCU_SCSS_PLL)) {
	}
}

/*!
 \brief   update the SystemCoreClock with current core clock retrieved from cpu registers
 \param   none
 \retval  none
*/
void SystemCoreClockUpdate(void)
{
	uint32_t tmp_val = 0U;
	uint32_t pllsel = 0U, pllpresel = 0U, predv0sel = 0U, pllmf = 0U, ck_src = 0U;

#ifdef GD32F30X_CL
	uint32_t predv0 = 0U, predv1 = 0U, pll1mf = 0U;
#endif /* GD32F30X_CL */

	tmp_val = GET_BITS(RCU_CFG0, 2, 3);

	switch (tmp_val) {
		case SEL_IRC8M: /* IRC8M is selected as CK_SYS */
			SystemCoreClock = IRC8M_VALUE;
			break;
		case SEL_HXTAL: /* HXTAL is selected as CK_SYS */
			SystemCoreClock = HXTAL_VALUE;
			break;
		case SEL_PLL:   /* PLL is selected as CK_SYS */
			/* PLL clock source selection, HXTAL, IRC48M or IRC8M / 2 */
			pllsel = (RCU_CFG0 & RCU_CFG0_PLLSEL);
			if (pllsel == RCU_PLLSRC_HXTAL_IRC48M) {
				/* PLL clock source is HXTAL or IRC48M */
				pllpresel = (RCU_CFG1 & RCU_CFG1_PLLPRESEL);
				if (pllpresel == RCU_PLLPRESRC_HXTAL) {
					/* PLL clock source is HXTAL */
					ck_src = HXTAL_VALUE;
				} else {
					/* PLL clock source is IRC48 */
					ck_src = IRC48M_VALUE;
				}
#if (defined(GD32F30X_HD) || defined(GD32F30X_XD))
				predv0sel = (RCU_CFG0 & RCU_CFG0_PREDV0);
				/* PREDV0 input source clock divided by 2 */
				if (predv0sel == RCU_CFG0_PREDV0) {
					ck_src /= 2U;
				}
#elif defined(GD32F30X_CL)
				predv0sel = (RCU_CFG1 & RCU_CFG1_PREDV0SEL);
				/* source clock use PLL1 */
				if (predv0sel == RCU_PREDV0SRC_CKPLL1) {
					predv1 = ((RCU_CFG1 & RCU_CFG1_PREDV1) >> 4) + 1U;
					pll1mf = ((RCU_CFG1 & RCU_CFG1_PLL1MF) >> 8) + 2U;
					if (pll1mf == 17U) {
						pll1mf = 20U;
					}
					ck_src = (ck_src / predv1) * pll1mf;
				}
				predv0 = (RCU_CFG1 & RCU_CFG1_PREDV0) + 1U;
				ck_src /= predv0;
#endif /* GD32F30X_HD and GD32F30X_XD */
			} else {
				/* PLL clock source is IRC8M / 2 */
				ck_src = IRC8M_VALUE / 2U;
			}
			/* PLL multiplication factor */
			pllmf = GET_BITS(RCU_CFG0, 18, 21);
			if ((RCU_CFG0 & RCU_CFG0_PLLMF_4)) {
				pllmf |= 0x10U;
			}
			if ((RCU_CFG0 & RCU_CFG0_PLLMF_5)) {
				pllmf |= 0x20U;
			}
			if (pllmf >= 15U) {
				pllmf += 1U;
			} else {
				pllmf += 2U;
			}
			if (pllmf > 61U) {
				pllmf = 63U;
			}
			SystemCoreClock = ck_src * pllmf;
		#ifdef GD32F30X_CL
			if (pllmf == 15U) {
				SystemCoreClock = (ck_src * 6U) + (ck_src / 2U);
			}
	#endif /* GD32F30X_CL */
			break;
		/* IRC8M is selected as CK_SYS */
		default:
			SystemCoreClock = IRC8M_VALUE;
			break;
	}

	/* calculate AHB clock frequency */
	tmp_val = AHBPrescaler[((RCU_CFG0 & RCU_CFG0_AHBPSC) >> 4U)];
	SystemCoreClock >>= tmp_val;
}
