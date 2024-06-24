/*!
   \file  systick.h
   \brief the header file of systick

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

#ifndef SYSTICK_H
#define SYSTICK_H

#include "gd32_def.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  TICK_OK,
  TICK_ERROR,
  TICK_TIMEOUT
} TICK_error_t;

typedef enum {
  SYSTICK_FREQ_10HZ = 100U,
  SYSTICK_FREQ_100HZ = 10U,
  SYSTICK_FREQ_1KHZ = 1U,
  SYSTICK_FREQ_DEFAULT = SYSTICK_FREQ_1KHZ
} systick_freq_t;

#define MAX_TICK_DELAY    0xFFFFFFFFU

extern uint32_t msTickPrio;
extern systick_freq_t msTickFreq;

/* configure systick */
TICK_error_t tickInit(uint32_t systick_priority);
void tickInc(void);
uint32_t getTickPrio(void);
TICK_error_t setTickFreq(systick_freq_t freq);
systick_freq_t getTickFreq(void);
void tickDelay(uint32_t delay);
uint32_t getTickMs(void);
uint32_t getTickUs(void);

/**
  * @brief  This function checks if the Systick counter flag is active
  * @retval State of bit (1 or 0).
  */
//static inline uint32_t systick_active_counter_flag(void)
//{
//  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
//}

#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H */
