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
#include "safe_clocks.h"

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
const uint32_t analog_pins[] = {
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

#ifdef __cplusplus
extern "C" {
#endif

#if defined(SAFE_CLOCKS_ENABLE)

static void set_pmu_output_voltage(void);
static void set_pmu_highdrive(void);

WEAK void SystemClock_Config(void)
{
  SC_oscillator_params_t osc_params = {};
  SC_clock_params_t clock_params = {};
  SC_peripheral_params_t pclk_params = {};

  set_pmu_output_voltage();

  osc_params.osc = RCU_OSC_HXTAL;
  osc_params.HXTAL_state = RCU_HXTAL_ON;
  osc_params.HXTAL_prediv = RCU_HXTAL_PREDIV2;
  //osc_params.HXTAL_prediv = RCU_HXTAL_PREDIV1;
  osc_params.IRC8M_state = RCU_IRC8M_ON;
  osc_params.pll_params.pll_status = RCU_PLL_ON;
  osc_params.pll_params.pll_source_clock = RCU_PLLSRC_HXTAL_IRC48M;
  osc_params.pll_params.pll_multiplier = RCU_PLL_MUL30;
  //osc_params.pll_params.pll_multiplier = RCU_PLL_MUL9;
  if (SC_Osc_Params(&osc_params) != SC_OK) {
    Error_Handler();
  }

  clock_params.clock = RCU_CLK_AHB | RCU_CLK_SYS | RCU_CLK_APB1 | RCU_CLK_APB2;
  clock_params.system_source = RCU_CKSYSSRC_PLL;
  clock_params.ahbclk_div = RCU_AHB_CKSYS_DIV1;
  clock_params.apb1clk_div = RCU_APB1_CKAHB_DIV2;
  clock_params.apb2clk_div = RCU_APB2_CKAHB_DIV1;
  if (SC_Clock_Params(&clock_params, WS_WSCNT_2) != SC_OK) {
    Error_Handler();
  }

  set_pmu_highdrive();

  pclk_params.pclock = RCU_PERIPHCLK_ADC;
  //pclk_params.adc_clk = RCU_CKADC_CKAPB2_DIV8;
  pclk_params.adc_clk = RCU_CKADC_CKAPB2_DIV4;
  if (SC_Periph_Params(&pclk_params) != SC_OK) {
    Error_Handler();
  }
}

static void set_pmu_output_voltage(void)
{
  uint32_t reg = PMU_CTL;
  reg &= ~PMU_CTL_LDOVS;
  reg |= (3 << 14);
  PMU_CTL = reg;
}

static void set_pmu_highdrive(void)
{
  if (((PMU_CS & (PMU_CS_HDRF)) >> 16) == 0U) {
    //do {
      //__IO uint32_t read;
      uint32_t reg = PMU_CTL;
      reg &= ~PMU_CTL_HDEN;
      reg |= PMU_CTL_HDEN;
      PMU_CTL = reg;
      //read = (PMU_CS & (PMU_CS_HDRF));
    //} while (0U);
  }

  if (((PMU_CS & (PMU_CS_HDSRF)) >> 17) == 0U) {
    //do {
      //__IO uint32_t read;
      uint32_t reg = PMU_CTL;
      reg &= ~PMU_CTL_HDS;
      reg |= PMU_CTL_HDS;
      //read = (PMU_CS & (PMU_CS_HDSRF));
    //} while (0U);
  }
}

#endif /* SAFE_CLOCKS_ENABLE */

#ifdef __cplusplus
}
#endif