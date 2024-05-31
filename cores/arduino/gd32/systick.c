/*!
  \file  systick.c
  \brief the systick configuration file

  \version 2017-02-10, V1.0.0, firmware for GD32F30x
  \version 2018-10-10, V1.1.0, firmware for GD32F30x
  \version 2018-12-25, V2.0.0, firmware for GD32F30x
*/

/*
  Copyright (c) 2018, GigaDevice Semiconductor Inc.

  All rights reserved.

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

#include "backup_domain.h"
#include "systick.h"

#ifdef __cplusplus
extern "C" {
#endif

__IO uint32_t msTicks;
uint32_t msTickPrio = (1UL << __NVIC_PRIO_BITS);
systick_freq_t msTickFreq = SYSTICK_FREQ_DEFAULT;

/*!
  \brief      configure systick
  \param[in]  none
  \param[out] none
  \retval     none
*/
SC_error_t systick_init(uint32_t systick_priority)
{
  /* setup systick timer for 1000Hz interrupts */
  if (SysTick_Config(SystemCoreClock / (1000U / msTickFreq)) > 0U) {
    return SC_ERROR;
  }

  if (systick_priority < (1UL << __NVIC_PRIO_BITS)) {
    uint32_t priority_group = NVIC_GetPriorityGrouping();
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(priority_group, systick_priority, 0U));
    msTickPrio = systick_priority;
  } else {
    return SC_ERROR;
  }

  return SC_OK;
}

void systick_increase(void)
{
  msTicks += msTickFreq; // 1ms increases
}

void noOsSystickHandler(){}

void osSystickHandler() __attribute__((weak, alias("noOsSystickHandler")));

/*!
  \brief      this function handles SysTick exception
  \param[in]  none
  \retval     none
*/
void SysTick_Handler(void)
{
  systick_increase();
  osSystickHandler();
}

/*!
  \brief      get current milliseconds
  \param[in]  none
  \param[out] none
  \retval     current milliseconds
*/
uint32_t getCurrentMillis(void)
{
  return msTicks;
}

/*!
  \brief      get current microseconds
  \param[in]  none
  \param[out] none
  \retval     current microseconds
*/
uint32_t getCurrentMicros(void)
{
  systick_active_counter_flag();
  uint32_t ms = msTicks;
  const uint32_t systick_load = SysTick->LOAD + 1;
  __IO uint32_t us = systick_load - SysTick->VAL;
  if (systick_active_counter_flag()) {
    ms = msTicks;
    us = systick_load - SysTick->VAL;
  }
  return (ms * 1000 + (us * 1000) / systick_load);
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

#ifdef __cplusplus
}
#endif
