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
*/

#include "timer.h"
#ifdef GD32E50x
#include <gd32e50x_rcu.h>
#include <gd32e50x_timer.h>
#endif
#ifdef GD32F30x
#include <gd32f30x_timer.h>
#include <gd32f30x_rcu.h>
#endif

#if defined(GD32F1x0) || defined(GD32F3x0) || defined(GD32E50X) || defined(GD32EPRT)
#define TIMER5_IRQ_Name TIMER5_DAC_IRQn
#else
#define TIMER5_IRQ_Name TIMER5_IRQn
#endif

#if defined(GD32E23x) || defined(GD32F1x0) || defined(GD32F3x0) || defined(GD32E50X)
#define TIMER0_IRQ_Name TIMER0_Channel_IRQn
#else
#define TIMER0_IRQ_Name TIMER0_IRQn
#endif

#if defined(GD32F30x)
#if defined(GD32F30X_HD)
#define NO_TIMER_8
#define NO_TIMER_9
#define NO_TIMER_10
#define NO_TIMER_11
#define NO_TIMER_12
#define NO_TIMER_13
#define TIMER0_Update_IRQ_Name TIMER0_UP_IRQn
#else
#define TIMER0_Update_IRQ_Name TIMER0_UP_TIMER9_IRQn
#endif
#elif defined(GD32E23x) || defined(GD32F3x0) || defined(GD32F1x0)
#define TIMER0_Update_IRQ_Name TIMER0_BRK_UP_TRG_COM_IRQn
#elif defined(GD32E50X)
#if defined(GD32E50X_HD) || defined(GD32EPRT)
#define TIMER0_Update_IRQ_Name TIMER0_UP_IRQn
#define NO_TIMER_9
#else
#define TIMER0_Update_IRQ_Name TIMER0_UP_TIMER9_IRQn
#define TIMER9_IRQ_NAME TIMER0_UP_TIMER9_IRQn
#endif
#elif defined(GD32F10x)
// GD32F10X_XD has these, but requires work to add
#define NO_TIMER_8
#define NO_TIMER_9
#define NO_TIMER_10
#define NO_TIMER_11
#define NO_TIMER_12
#define NO_TIMER_13
#if !defined(GD32F10X_XD)
#define TIMER0_Update_IRQ_Name TIMER0_UP_IRQn
#else
#define TIMER0_Update_IRQ_Name TIMER0_UP_TIMER9_IRQn
#endif
#endif

#if defined(GD32E50X)
#define TIMER7_IRQ_NAME TIMER7_Channel_IRQn
#if defined(GD32E50X_XD) || defined(GD32E50X_CL) || defined(GD32E508)
#define TIMER7_UP_IRQ_NAME TIMER7_UP_TIMER12_IRQn
#define TIMER10_IRQ_NAME TIMER0_TRG_CMT_TIMER10_IRQn
#define TIMER11_IRQ_NAME TIMER7_BRK_TIMER11_IRQn
#define TIMER12_IRQ_NAME TIMER7_UP_TIMER12_IRQn
#define TIMER13_IRQ_NAME TIMER7_TRG_CMT_TIMER13_IRQn
#else 
#define TIMER7_UP_IRQ_NAME TIMER7_UP_IRQn
#define NO_TIMER_10
#define NO_TIMER_11
#define NO_TIMER_12
#define NO_TIMER_13
#endif
#elif defined(GD32F10x)
#if !defined(GD32F10X_XD)
#define TIMER7_IRQ_NAME TIMER7_UP_IRQn
#define TIMER7_UP_IRQ_NAME TIMER7_UP_IRQn
#else
#define TIMER7_IRQ_NAME TIMER7_UP_TIMER12_IRQn
#define TIMER7_UP_IRQ_NAME TIMER7_UP_TIMER12_IRQn
#endif
#else
#define TIMER7_IRQ_NAME TIMER7_IRQn
#define TIMER7_UP_IRQ_NAME TIMER7_IRQn
#endif
/* availablility of timers despite their macros being defined. */
/* e.g., for a GD32F303CC, the macro TIMER8 is defined, although */
/* TIMER8_IRQn is not defined since the timer is not actually available */ 
/* we have to work around this. */
#if defined(GD32F30x)
#if !defined(GD32F30X_HD)
#define HAS_TIMER_8
#define HAS_TIMER_9
#define HAS_TIMER_10
#define HAS_TIMER_11
#define HAS_TIMER_12
#define HAS_TIMER_13
#endif
#else
#ifndef NO_TIMER_8
#define HAS_TIMER_8
#endif
#ifndef NO_TIMER_9
#define HAS_TIMER_9
#endif
#ifndef NO_TIMER_10
#define HAS_TIMER_10
#endif
#ifndef NO_TIMER_11
#define HAS_TIMER_11
#endif
#ifndef NO_TIMER_12
#define HAS_TIMER_12
#endif
#ifndef NO_TIMER_13
#define HAS_TIMER_13
#endif
#endif

#if defined(GD32F3x0) && !defined(GD32F350)
#define NO_TIMER_5
#endif

timerDevice_t *get_timer_object(SPL_TimerHandle_t *timer_handle)
{
  timerDevice_t *timerObj;
  timerObj = (timerDevice_t *)((char *)timer_handle - offsetof(timerDevice_t, handle));
  return (timerObj);
}

/*!
  \brief      enable timer clock
  \param[in]  instance: TIMERx(x=0..13)
  \param[out] none
  \retval     none
*/
void Timer_clock_enable(SPL_TimerHandle_t *timer_handle)
{
  uint32_t temp = 0;
  switch (timer_handle->timer_instance) {
#if defined(TIMER0)
    case TIMER0:
      temp = RCU_TIMER0;
      break;
#endif
#if defined(TIMER1)
    case TIMER1:
      temp = RCU_TIMER1;
      break;
#endif
#if defined(TIMER2)
    case TIMER2:
      temp = RCU_TIMER2;
      break;
#endif
#if defined(TIMER3)
    case TIMER3:
      temp = RCU_TIMER3;
      break;
#endif
#if defined(TIMER4)
    case TIMER4:
      temp = RCU_TIMER4;
      break;
#endif
#if defined(TIMER5) && !defined(NO_TIMER_5)
    case TIMER5:
      temp = RCU_TIMER5;
      break;
#endif
#if defined(TIMER6)
    case TIMER6:
      temp = RCU_TIMER6;
      break;
#endif
#if defined(TIMER7)
    case TIMER7:
      temp = RCU_TIMER7;
      break;
#endif
#if defined(TIMER8)&& defined(HAS_TIMER_8) /* ToDO: Fix this so for non-F30x series that also have TIMER8 *and* RCU_TIMER8 */
    case TIMER8:
      temp = RCU_TIMER8;
      break;
#endif
#if defined(TIMER9)&& defined(HAS_TIMER_9)
    case TIMER9:
      temp = RCU_TIMER9;
      break;
#endif
#if defined(TIMER10)&& defined(HAS_TIMER_10) 
    case TIMER10:
      temp = RCU_TIMER10;
      break;
#endif
#if defined(TIMER11)&& defined(HAS_TIMER_11)
    case TIMER11:
      temp = RCU_TIMER11;
      break;
#endif
#if defined(TIMER12)&& defined(HAS_TIMER_12)
    case TIMER12:
      temp = RCU_TIMER12;
      break;
#endif
#if defined(TIMER13)&& defined(HAS_TIMER_13)
    case TIMER13:
      temp = RCU_TIMER13;
      break;
#endif
#if defined(TIMER14)
    case TIMER14:
      temp = RCU_TIMER14;
      break;
#endif
#if defined(TIMER15)
    case TIMER15:
      temp = RCU_TIMER15;
      break;
#endif
#if defined(TIMER16)
    case TIMER16:
      temp = RCU_TIMER16;
      break;
#endif
    default:
      break;
  }
  rcu_periph_clock_enable(temp);
}

/*!
  \brief      disable timer clock
  \param[in]  instance: TIMERx(x=0..13)
  \param[out] none
  \retval     none
*/
void Timer_clock_disable(SPL_TimerHandle_t *timer_handle)
{
  uint32_t temp = 0;
  switch (timer_handle->timer_instance) {
#if defined(TIMER0)
    case TIMER0:
      temp = RCU_TIMER0;
      break;
#endif
#if defined(TIMER1)
    case TIMER1:
      temp = RCU_TIMER1;
      break;
#endif
#if defined(TIMER2)
    case TIMER2:
      temp = RCU_TIMER2;
      break;
#endif
#if defined(TIMER3)
    case TIMER3:
      temp = RCU_TIMER3;
      break;
#endif
#if defined(TIMER4)
    case TIMER4:
      temp = RCU_TIMER4;
      break;
#endif
#if defined(TIMER5) && !defined(NO_TIMER_5)
    case TIMER5:
      temp = RCU_TIMER5;
      break;
#endif
#if defined(TIMER6)
    case TIMER6:
      temp = RCU_TIMER6;
      break;
#endif
#if defined(TIMER7)
    case TIMER7:
      temp = RCU_TIMER7;
      break;
#endif
#if defined(TIMER8)&& defined(HAS_TIMER_8) /* ToDO: Fix this so for non-F30x series that also have TIMER8 *and* RCU_TIMER8 */
    case TIMER8:
      temp = RCU_TIMER8;
      break;
#endif
#if defined(TIMER9)&& defined(HAS_TIMER_9)
    case TIMER9:
      temp = RCU_TIMER9;
      break;
#endif
#if defined(TIMER10)&& defined(HAS_TIMER_10) 
    case TIMER10:
      temp = RCU_TIMER10;
      break;
#endif
#if defined(TIMER11)&& defined(HAS_TIMER_11)
    case TIMER11:
      temp = RCU_TIMER11;
      break;
#endif
#if defined(TIMER12)&& defined(HAS_TIMER_12)
    case TIMER12:
      temp = RCU_TIMER12;
      break;
#endif
#if defined(TIMER13)&& defined(HAS_TIMER_13)
    case TIMER13:
      temp = RCU_TIMER13;
      break;
#endif
#if defined(TIMER14)
    case TIMER14:
      temp = RCU_TIMER14;
      break;
#endif
#if defined(TIMER15)
    case TIMER15:
      temp = RCU_TIMER15;
      break;
#endif
#if defined(TIMER16)
    case TIMER16:
      temp = RCU_TIMER16;
      break;
#endif
    default:
      break;
  }
  rcu_periph_clock_disable(temp);
}

/*!
  \brief      initialize timer
  \param[in]  instance: TIMERx(x=0..13)
  \param[in]  timerPeriod: period and format
  \param[out] none
  \retval     none
*/
void Timer_init(SPL_TimerHandle_t *timer_handle)
{
  if (timer_handle == NULL)
    return;

  timerDevice_t *timerObj = get_timer_object(timer_handle);
  Timer_clock_enable(timer_handle);
  timer_deinit(timer_handle->timer_instance);

#if defined(GD32E23x)
  /* no subpriority */
  nvic_irq_enable(getTimerUpIrq(timer_handle->timer_instance), timerObj->timerPreemptPriority);
  nvic_irq_enable(getTimerCCIrq(timer_handle->timer_instance), timerObj->timerPreemptPriority);
#else
  nvic_irq_enable(getTimerUpIrq(timer_handle->timer_instance), timerObj->timerPreemptPriority, timerObj->timerSubPriority);
  nvic_irq_enable(getTimerCCIrq(timer_handle->timer_instance), timerObj->timerPreemptPriority, timerObj->timerSubPriority);
#endif
  timer_init(timer_handle->timer_instance, &timer_handle->init_params);
}

/*!
  \brief      enable timer update interrupt
  \param[in]  instance: TIMERx(x=0..13)
  \param[out] none
  \retval     none
*/
void Timer_enableUpdateIT(uint32_t instance)
{
  timer_flag_clear(instance, TIMER_INT_FLAG_UP);
  timer_interrupt_enable(instance, TIMER_INT_UP);
}

/*!
  \brief      disable timer update interrupt
  \param[in]  instance: TIMERx(x=0..13)
  \param[out] none
  \retval     none
*/
void Timer_disableUpdateIT(uint32_t instance)
{
  timer_flag_clear(instance, TIMER_INT_FLAG_UP);
  timer_interrupt_disable(instance, TIMER_INT_UP);
}

/*!
  \brief      enable timer channel capture interrupt
  \param[in]  instance: TIMERx(x=0..13)
  \param[in]  interrupt: the channel's interrupt
  \param[out] none
  \retval     none
*/
void Timer_enableCaptureIT(uint32_t instance, uint32_t interrupt)
{
  uint32_t interrupt_flag;
  switch (interrupt) {
    case TIMER_INT_CH0:
      interrupt_flag = TIMER_INT_FLAG_CH0;
      break;
    case TIMER_INT_CH1:
      interrupt_flag = TIMER_INT_FLAG_CH1;
      break;
    case TIMER_INT_CH2:
      interrupt_flag = TIMER_INT_FLAG_CH2;
      break;
    case TIMER_INT_CH3:
      interrupt_flag = TIMER_INT_FLAG_CH3;
      break;
    default:
      //ToDo: better error handling in case of invalid params
      return;
  }
  timer_interrupt_flag_clear(instance, interrupt_flag);
  timer_interrupt_enable(instance, interrupt);
}

/*!
  \brief      disable timer channel capture interrupt
  \param[in]  instance: TIMERx(x=0..13)
  \param[in]  interrupt: the channel's interrupt
  \param[out] none
  \retval     none
*/
void Timer_disableCaptureIT(uint32_t instance, uint32_t interrupt)
{
  uint32_t interrupt_flag;
  switch (interrupt) {
    case TIMER_INT_CH0:
      interrupt_flag = TIMER_INT_FLAG_CH0;
      break;
    case TIMER_INT_CH1:
      interrupt_flag = TIMER_INT_FLAG_CH1;
      break;
    case TIMER_INT_CH2:
      interrupt_flag = TIMER_INT_FLAG_CH2;
      break;
    case TIMER_INT_CH3:
      interrupt_flag = TIMER_INT_FLAG_CH3;
      break;
    default:
      //ToDo: better error handling in case of invalid params
      return;
  }
  timer_interrupt_flag_clear(instance, interrupt_flag);
  timer_interrupt_disable(instance, interrupt);
}

/*!
  \brief      get timer clock frequency
  \param[in]  instance: TIMERx(x=0..13)
  \param[out] none
  \retval     none
*/
uint32_t getTimerClkFrequency(uint32_t instance)
{
  rcu_clock_freq_enum timerclkSrc = 0xf;
  uint32_t APBx_PSC = 0;
  uint32_t clk_src = 0;

  if (instance != -1) {
    switch ((uint32_t)instance) {
#if defined(TIMER0)
      case (uint32_t)TIMER0:
#endif
#if defined(TIMER7)
      case (uint32_t)TIMER7:
#endif
#if defined(TIMER8)
      case (uint32_t)TIMER8:
#endif
#if defined(TIMER9)
      case (uint32_t)TIMER9:
#endif
#if defined(TIMER10)
      case (uint32_t)TIMER10:
#endif
        timerclkSrc = CK_APB2;
        APBx_PSC = (RCU_CFG0 & RCU_CFG0_APB2PSC) >> 11;
        break;
#if defined(TIMER1)
      case (uint32_t)TIMER1:
#endif
#if defined(TIMER2)
      case (uint32_t)TIMER2:
#endif
#if defined(TIMER3)
      case (uint32_t)TIMER3:
#endif
#if defined(TIMER4)
      case (uint32_t)TIMER4:
#endif
#if defined(TIMER5) && !defined(NO_TIMER_5)
      case (uint32_t)TIMER5:
#endif
#if defined(TIMER6)
      case (uint32_t)TIMER6:
#endif
#if defined(TIMER11)
      case (uint32_t)TIMER11:
#endif
#if defined(TIMER12)
      case (uint32_t)TIMER12:
#endif
#if defined(TIMER13)
      case (uint32_t)TIMER13:
#endif
#if defined(TIMER14)
      case (uint32_t)TIMER14:
#endif
#if defined(TIMER15)
      case (uint32_t)TIMER15:
#endif
#if defined(TIMER16)
      case (uint32_t)TIMER16:
#endif
        timerclkSrc = CK_APB1;
        APBx_PSC = (RCU_CFG0 & RCU_CFG0_APB1PSC) >> 8;
        break;
      default:
        break;
    }
  }
  if (0 != (APBx_PSC & 0x04)) {
    clk_src = 2 * rcu_clock_freq_get(timerclkSrc);
  } else {
    clk_src = rcu_clock_freq_get(timerclkSrc);
  }

  return clk_src;
}

/*!
  \brief      get timer update IRQn
  \param[in]  tim: TIMERx(x=0..13)
  \param[out] none
  \retval     timer update IRQn
*/
IRQn_Type getTimerUpIrq(uint32_t tim)
{
  IRQn_Type IRQn = NonMaskableInt_IRQn;

  if (tim != (uint32_t)NC) {
    switch ((uint32_t)tim) {
#if defined(TIMER0)
      case (uint32_t)TIMER0:
        //differing update interrupt for Timer0 on some devices
        IRQn = TIMER0_Update_IRQ_Name;
        break;
#endif
#if defined(TIMER1)
      case (uint32_t)TIMER1:
        IRQn = TIMER1_IRQn;
        break;
#endif
#if defined(TIMER2)
      case (uint32_t)TIMER2:
        IRQn = TIMER2_IRQn;
        break;
#endif
#if defined(TIMER3)
      case (uint32_t)TIMER3:
        IRQn = TIMER3_IRQn;
        break;
#endif
#if defined(TIMER4)
      case (uint32_t)TIMER4:
        IRQn = TIMER4_IRQn;
        break;
#endif
#if defined(TIMER5) && !defined(NO_TIMER_5)
      case (uint32_t)TIMER5:
        IRQn = TIMER5_IRQ_Name;
        break;
#endif
#if defined(TIMER6)
      case (uint32_t)TIMER6:
        IRQn = TIMER6_IRQn;
        break;
#endif
#if defined(TIMER7)
      case (uint32_t)TIMER7:
        IRQn = TIMER7_UP_IRQ_NAME;
        break;
#endif
#if defined(TIMER8) && defined(HAS_TIMER_8)
      case (uint32_t)TIMER8:
        IRQn = TIMER8_IRQn;
        break;
#endif
#if defined(TIMER9) && defined(HAS_TIMER_9)
      case (uint32_t)TIMER9:
        IRQn = TIMER9_IRQn;
        break;
#endif
#if defined(TIMER10) && defined(HAS_TIMER_10)
      case (uint32_t)TIMER10:
      #ifdef TIMER10_IRQ_NAME //TODO: repeat this for other timers...
        IRQn = TIMER10_IRQ_NAME;
      #else
        IRQn = TIMER10_IRQn;
      #endif
        break;
#endif
#if defined(TIMER11) && defined(HAS_TIMER_11)
      case (uint32_t)TIMER11:
        IRQn = TIMER11_IRQn;
        break;
#endif
#if defined(TIMER12) && defined(HAS_TIMER_12)
      case (uint32_t)TIMER12:
        IRQn = TIMER12_IRQn;
        break;
#endif
#if defined(TIMER13) && defined(HAS_TIMER_13)
      case (uint32_t)TIMER13:
        IRQn = TIMER13_IRQn;
        break;
#endif
      default:
        break;
    }
  }
  return IRQn;
}

/*!
  \brief      get timer capture/compare IRQn
  \param[in]  tim: TIMERx(x=0..13)
  \param[out] none
  \retval     timer capture/compare IRQn
*/
IRQn_Type getTimerCCIrq(uint32_t tim)
{
  IRQn_Type IRQn = NonMaskableInt_IRQn;

  if (tim != (uint32_t)NC) {
    switch ((uint32_t)tim) {
#if defined(TIMER0)
      case (uint32_t)TIMER0:
        IRQn = TIMER0_Channel_IRQn;
        break;
#endif
#if defined(TIMER1)
      case (uint32_t)TIMER1:
        IRQn = TIMER1_IRQn;
        break;
#endif
#if defined(TIMER2)
      case (uint32_t)TIMER2:
        IRQn = TIMER2_IRQn;
        break;
#endif
#if defined(TIMER3)
      case (uint32_t)TIMER3:
        IRQn = TIMER3_IRQn;
        break;
#endif
#if defined(TIMER4)
      case (uint32_t)TIMER4:
        IRQn = TIMER4_IRQn;
        break;
#endif
#if defined(TIMER5) && !defined(NO_TIMER_5)
      case (uint32_t)TIMER5:
        IRQn = TIMER5_IRQ_Name;
        break;
#endif
#if defined(TIMER6)
      case (uint32_t)TIMER6:
        IRQn = TIMER6_IRQn;
        break;
#endif
#if defined(TIMER7)
      case (uint32_t)TIMER7:
        IRQn = TIMER7_IRQ_NAME;
        break;
#endif
#if defined(TIMER8) && defined(HAS_TIMER_8)
      case (uint32_t)TIMER8:
        IRQn = TIMER8_IRQn;
        break;
#endif
#if defined(TIMER9) && defined(HAS_TIMER_9)
      case (uint32_t)TIMER9:
        IRQn = TIMER9_IRQn;
        break;
#endif
#if defined(TIMER10) && defined(HAS_TIMER_10)
      case (uint32_t)TIMER10:
        IRQn = TIMER10_IRQn;
        break;
#endif
#if defined(TIMER11) && defined(HAS_TIMER_11)
      case (uint32_t)TIMER11:
        IRQn = TIMER11_IRQn;
        break;
#endif
#if defined(TIMER12) && defined(HAS_TIMER_12)
      case (uint32_t)TIMER12:
        IRQn = TIMER12_IRQn;
        break;
#endif
#if defined(TIMER13) && defined(HAS_TIMER_13)
      case (uint32_t)TIMER13:
        IRQn = TIMER13_IRQn;
        break;
#endif
#if defined(TIMER14)
      case (uint32_t)TIMER14:
        IRQn = TIMER14_IRQn;
        break;
#endif
#if defined(TIMER15)
      case (uint32_t)TIMER15:
        IRQn = TIMER15_IRQn;
        break;
#endif
#if defined(TIMER16)
      case (uint32_t)TIMER16:
        IRQn = TIMER16_IRQn;
        break;
#endif
      default:
        break;
    }
  }
  return IRQn;
}

/*!
  \brief      timer interrupt handler
  \param[in]  timer_handle: SPL timer handle
  \param[out] none
  \retval     none
*/
void timerInterruptHandler(SPL_TimerHandle_t *timer_handle)
{
  if (timer_interrupt_flag_get(timer_handle->timer_instance, TIMER_INT_FLAG_UP) != RESET) {
    timer_interrupt_flag_clear(timer_handle->timer_instance, TIMER_INT_FLAG_UP);
    timer_updateHandle(timer_handle);
  }

  if (timer_interrupt_flag_get(timer_handle->timer_instance, TIMER_INT_FLAG_CH0) != RESET) {
    timer_interrupt_flag_clear(timer_handle->timer_instance, TIMER_INT_FLAG_CH0);
    timer_handle->Channel = CHAN0_ACTIVE;
    timer_captureHandle(timer_handle);
    timer_handle->Channel = CHAN_ALL_CLEARED;
  }

  if (timer_interrupt_flag_get(timer_handle->timer_instance, TIMER_INT_FLAG_CH1) != RESET) {
    timer_interrupt_flag_clear(timer_handle->timer_instance, TIMER_INT_FLAG_CH1);
    timer_handle->Channel = CHAN1_ACTIVE;
    timer_captureHandle(timer_handle);
    timer_handle->Channel = CHAN_ALL_CLEARED;
  }

  if (timer_interrupt_flag_get(timer_handle->timer_instance, TIMER_INT_FLAG_CH2) != RESET) {
    timer_interrupt_flag_clear(timer_handle->timer_instance, TIMER_INT_FLAG_CH2);
    timer_handle->Channel = CHAN2_ACTIVE;
    timer_captureHandle(timer_handle);
    timer_handle->Channel = CHAN_ALL_CLEARED;
  }

  if (timer_interrupt_flag_get(timer_handle->timer_instance, TIMER_INT_FLAG_CH3) != RESET) {
    timer_interrupt_flag_clear(timer_handle->timer_instance, TIMER_INT_FLAG_CH3);
    timer_handle->Channel = CHAN3_ACTIVE;
    timer_captureHandle(timer_handle);
    timer_handle->Channel = CHAN_ALL_CLEARED;
  }
}

__attribute__((weak)) void timer_updateHandle(SPL_TimerHandle_t *timer_handle)
{

}

__attribute__((weak)) void timer_captureHandle(SPL_TimerHandle_t *timer_handle)
{

}
