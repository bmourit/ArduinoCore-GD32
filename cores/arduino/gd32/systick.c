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

__IO uint32_t msTicks;
uint32_t msTickPrio = (1UL << __NVIC_PRIO_BITS);
systick_freq_t msTickFreq = SYSTICK_FREQ_DEFAULT;

/*!
  \brief      configure systick
  \param[in]  priority
  \param[out] none
  \retval     none
*/
TICK_error_t tickInit(uint32_t priority)
{
  if (SysTick_Config(SystemCoreClock / (1000U / msTickFreq)) > 0U) {
    return TICK_ERROR;
  }

  if (priority < (1UL << __NVIC_PRIO_BITS)) {
    NVIC_SetPriority(SysTick_IRQn, priority);
    msTickPrio = priority;
  } else {
    return TICK_ERROR;
  }
  return TICK_OK;
}

void tickInc(void)
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
  tickInc();
  osSystickHandler();
}

/*!
  \brief      get the current tick priority
  \param[in]  none
  \param[out] none
  \retval     tick priority
*/
uint32_t getTickPrio(void)
{
  return msTickPrio;
}

TICK_error_t setTickFreq(systick_freq_t freq)
{
  TICK_error_t state = TICK_OK;
  if (freq != msTickFreq) {
    state = tickInit(msTickPrio);
    if (state == TICK_OK) {
      msTickFreq = freq;
    }
  }
  return state;
}

systick_freq_t getTickFreq(void)
{
  return msTickFreq;
}

void tickDelay(uint32_t delay)
{
  uint32_t tstart = getTickMs();
  uint32_t tdelay = delay;

  if (tdelay < MAX_TICK_DELAY) {
    tdelay += (uint32_t)(msTickFreq);
  }

  while ((getTickMs() - tstart) < tdelay) {
  }
}

/*!
  \brief      get tick value in milliseconds
  \param[in]  none
  \param[out] none
  \retval     current milliseconds
*/
uint32_t getTickMs(void)
{
  return msTicks;
}

/*!
  \brief      get current microseconds
  \param[in]  none
  \param[out] none
  \retval     current microseconds
*/
uint32_t getTickUs(void)
{
  uint32_t ms = msTicks;
  uint32_t us = SysTick->LOAD - SysTick->VAL;

  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
    ms = msTicks;
    us = SysTick->LOAD - SysTick->VAL;
  }

  return (ms * 1000 + (us * 1000) / SysTick->LOAD);
}
