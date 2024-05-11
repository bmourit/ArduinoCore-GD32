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

WEAK void SystemClock_Config(void)
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

  /* select HXTAL / 2 as clock source */
  RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
  RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_CFG0_PREDV0);

  /* CK_PLL = (CK_HXTAL / 2) * 30 = 120 MHz */
  RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
  RCU_CFG0 |= RCU_PLL_MUL30;

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

#ifdef __cplusplus
}
#endif
