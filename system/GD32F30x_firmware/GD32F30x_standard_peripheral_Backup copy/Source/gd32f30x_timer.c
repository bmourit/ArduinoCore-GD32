/*!
  \file    gd32f30x_timer.c
  \brief   TIMER driver

  \version 2017-02-10, V1.0.0, firmware for GD32F30x
  \version 2018-10-10, V1.1.0, firmware for GD32F30x
  \version 2018-12-25, V2.0.0, firmware for GD32F30x
  \version 2020-09-30, V2.1.0, firmware for GD32F30x
*/

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

#include "gd32f30x_timer.h"

/*!
  \brief      deinit a TIMER
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_deinit(timer_peripheral_t *timer_pdata)
{
  switch(timer_pdata->baseAddress) {
  case TIMER0_BASE:
    /* reset TIMER0 */
    rcu_periph_reset_enable(RCU_TIMER0RST);
    rcu_periph_reset_disable(RCU_TIMER0RST);
    break;
  case TIMER1_BASE:
    /* reset TIMER1 */
    rcu_periph_reset_enable(RCU_TIMER1RST);
    rcu_periph_reset_disable(RCU_TIMER1RST);
    break;
  case TIMER2_BASE:
    /* reset TIMER2 */
    rcu_periph_reset_enable(RCU_TIMER2RST);
    rcu_periph_reset_disable(RCU_TIMER2RST);
    break;
  case TIMER3_BASE:
    /* reset TIMER3 */
    rcu_periph_reset_enable(RCU_TIMER3RST);
    rcu_periph_reset_disable(RCU_TIMER3RST);
    break;
  case TIMER4_BASE:
    /* reset TIMER4 */
    rcu_periph_reset_enable(RCU_TIMER4RST);
    rcu_periph_reset_disable(RCU_TIMER4RST);
    break;
  case TIMER5_BASE:
    /* reset TIMER5 */
    rcu_periph_reset_enable(RCU_TIMER5RST);
    rcu_periph_reset_disable(RCU_TIMER5RST);
    break;
  case TIMER6_BASE:
    /* reset TIMER6 */
    rcu_periph_reset_enable(RCU_TIMER6RST);
    rcu_periph_reset_disable(RCU_TIMER6RST);
    break;
  case TIMER7_BASE:
    /* reset TIMER7 */
    rcu_periph_reset_enable(RCU_TIMER7RST);
    rcu_periph_reset_disable(RCU_TIMER7RST);
    break;
#ifndef GD32F30X_HD
  case TIMER8_BASE:
    /* reset TIMER8 */
    rcu_periph_reset_enable(RCU_TIMER8RST);
    rcu_periph_reset_disable(RCU_TIMER8RST);
    break;
  case TIMER9_BASE:
    /* reset TIMER9 */
    rcu_periph_reset_enable(RCU_TIMER9RST);
    rcu_periph_reset_disable(RCU_TIMER9RST);
    break;
  case TIMER10_BASE:
    /* reset TIMER10 */
    rcu_periph_reset_enable(RCU_TIMER10RST);
    rcu_periph_reset_disable(RCU_TIMER10RST);
    break;
  case TIMER11_BASE:
    /* reset TIMER11 */
    rcu_periph_reset_enable(RCU_TIMER11RST);
    rcu_periph_reset_disable(RCU_TIMER11RST);
    break;
  case TIMER12_BASE:
    /* reset TIMER12 */
    rcu_periph_reset_enable(RCU_TIMER12RST);
    rcu_periph_reset_disable(RCU_TIMER12RST);
    break;
  case TIMER13_BASE:
    /* reset TIMER13 */
    rcu_periph_reset_enable(RCU_TIMER13RST);
    rcu_periph_reset_disable(RCU_TIMER13RST);
    break;
#endif /* GD32F30X_HD */
  default:
    break;
  }
}

/*!
  \brief      initialize TIMER init parameter struct with a default value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_config_init(timer_peripheral_t *timer_pdata)
{
  /* initialize the parameter struct member with the default value */
  timer_pdata->timer_config->prescaler_value = 0U;
  timer_pdata->timer_config->aligned_mode = TIMER_COUNTER_EDGE;
  timer_pdata->timer_config->counter_direction = TIMER_COUNTER_UP;
  timer_pdata->timer_config->auto_reload_value = 65535U;
  timer_pdata->timer_config->clock_division = TIMER_CKDIV_DIV1;
  timer_pdata->timer_config->repetition_value = 0U;
}

/*!
  \brief      initialize TIMER counter
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_init(timer_peripheral_t *timer_pdata)
{
  /* configure the counter prescaler value */
  TIMER_PSC(timer_pdata->baseAddress) = (uint16_t)timer_pdata->timer_config->prescaler_value;

  /* configure the counter direction and aligned mode */
  if ((timer_pdata->baseAddress == TIMER0_BASE) || (timer_pdata->baseAddress == TIMER1_BASE) || (timer_pdata->baseAddress == TIMER2_BASE)
    || (timer_pdata->baseAddress == TIMER3_BASE) || (timer_pdata->baseAddress == TIMER4_BASE) || (timer_pdata->baseAddress == TIMER7_BASE)) {
    TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)(TIMER_CTL0_DIR | TIMER_CTL0_CAM);
    TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->timer_config->aligned_mode;
    TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)itimer_pdata->timer_config->counter_direction;
  }

  /* configure the autoreload value */
  TIMER_CAR(timer_pdata->baseAddress) = (uint32_t)timer_pdata->timer_config->auto_reload_value;

  if ((timer_pdata->baseAddress != TIMER5_BASE) && (timer_pdata->baseAddress != TIMER6_BASE)) {
    /* reset the CKDIV bit */
    TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL0_CKDIV;
    TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->timer_config->clock_division;
  }

  if ((timer_pdata->baseAddress == TIMER0_BASE) || (timer_pdata->baseAddress == TIMER7_BASE)) {
    /* configure the repetition counter value */
    TIMER_CREP(timer_pdata->baseAddress) = (uint32_t)timer_pdata->timer_config->repetition_value;
  }

  /* generate an update event */
  TIMER_SWEVG(timer_pdata->baseAddress) |= (uint32_t)TIMER_SWEVG_UPG;
}

/*!
  \brief      enable a TIMER
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL0_CEN;
}

/*!
  \brief      disable a TIMER
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL0_CEN;
}

/*!
  \brief      enable the auto reload shadow function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_auto_reload_shadow_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL0_ARSE;
}

/*!
  \brief      disable the auto reload shadow function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_auto_reload_shadow_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL0_ARSE;
}

/*!
  \brief      enable the update event
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_update_event_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL0_UPDIS;
}

/*!
  \brief      disable the update event
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_update_event_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t) TIMER_CTL0_UPDIS;
}

/*!
  \brief      set TIMER counter alignment mode
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_COUNTER_EDGE: edge-aligned mode
    \arg        TIMER_COUNTER_CENTER_DOWN: center-aligned and counting down assert mode
    \arg        TIMER_COUNTER_CENTER_UP: center-aligned and counting up assert mode
    \arg        TIMER_COUNTER_CENTER_BOTH: center-aligned and counting up/down assert mode
  \retval     none
*/
void timer_counter_alignment(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL0_CAM;
  TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->timer_config->aligned_mode;
}

/*!
  \brief      set TIMER counter up direction
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_counter_up_direction(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL0_DIR;
}

/*!
  \brief      set TIMER counter down direction
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_counter_down_direction(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL0_DIR;
}

/*!
  \brief      configure TIMER prescaler value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  prescaler: prescaler value,0~65535
  \param[in]  pscreload: prescaler reload mode
        only one parameter can be selected which is shown as below:
    \arg        TIMER_PSC_RELOAD_NOW: the prescaler is loaded right now
    \arg        TIMER_PSC_RELOAD_UPDATE: the prescaler is loaded at the next update event
  \retval     none
*/


void timer_prescaler_reload_now(timer_peripheral_t *timer_pdata)
{
  TIMER_PSC(timer_pdata->baseAddress) == timer_pdata->timer_config->prescaler_value;
  TIMER_SWEVG(timer_pdata->baseAddress) |= (uint32_t)TIMER_SWEVG_UPG;
}

void timer_prescaler_reload_update(timer_peripheral_t *timer_pdata)
{
  TIMER_PSC(timer_pdata->baseAddress) == timer_pdata->timer_config->prescaler_value;
}

/*!
  \brief      configure TIMER repetition register value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  repetition: the counter repetition value,0~255
  \retval     none
*/
void timer_repetition_value_set(timer_peripheral_t *timer_pdata)
{
  TIMER_CREP(timer_pdata->baseAddress) = (uint32_t)timer_pdata->timer_config->repetition_value;
} 
 
/*!
  \brief      configure TIMER autoreload register value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_autoreload_value_set(timer_peripheral_t *timer_pdata)
{
  TIMER_CAR(timer_pdata->baseAddress) = (uint32_t)timer_pdata->timer_config->auto_reload_value;
}

/*!
  \brief      configure TIMER counter register value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  counter: the counter value,0~65535
  \retval     none
*/
void timer_counter_value_set(timer_peripheral_t *timer_pdata, uint16_t counter_start_value)
{
  TIMER_CNT(timer_pdata->baseAddress) = (uint32_t)counter_start_value;
}

/*!
  \brief      read TIMER counter value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     counter value
*/         
uint32_t timer_counter_read(timer_peripheral_t *timer_pdata)
{
  return TIMER_CNT(timer_pdata->baseAddress);
}

/*!
  \brief      read TIMER prescaler value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     prescaler register value
*/
uint16_t timer_prescaler_read(timer_peripheral_t *timer_pdata)
{
  return (uint16_t)(TIMER_PSC(timer_pdata->baseAddress));
}

/*!
  \brief      configure TIMER in single pulse mode
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_set_single_pulse_mode(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL0_SPM;
}

/*!
  \brief      configure TIMER in repetitive pulse mode
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_set_repetitive_pulse_mode(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) &= ~((uint32_t)TIMER_CTL0_SPM);
}

/*!
  \brief      configure TIMER update source as global
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_update_source_set_global(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL0_UPS;
}

/*!
  \brief      configure TIMER update source as regular
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_update_source_set_regular(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL0_UPS;
}

/*!
  \brief      enable the TIMER interrupt
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  interrupt: timer interrupt enable source
        only one parameter can be selected which is shown as below:
    \arg        TIMER_INT_UP: update interrupt enable, TIMERx(x=0..13)
    \arg        TIMER_INT_CH0: channel 0 interrupt enable, TIMERx(x=0..4,7..13)
    \arg        TIMER_INT_CH1: channel 1 interrupt enable, TIMERx(x=0..4,7,8,11)
    \arg        TIMER_INT_CH2: channel 2 interrupt enable, TIMERx(x=0..4,7)
    \arg        TIMER_INT_CH3: channel 3 interrupt enable , TIMERx(x=0..4,7)
    \arg        TIMER_INT_CMT: commutation interrupt enable, TIMERx(x=0,7)
    \arg        TIMER_INT_TRG: trigger interrupt enable, TIMERx(x=0..4,7,8,11)
    \arg        TIMER_INT_BRK: break interrupt enable, TIMERx(x=0,7)
  \retval     none
*/
void timer_interrupt_enable(timer_peripheral_t *timer_pdata, uint32_t interrupt)
{
  TIMER_DMAINTEN(timer_pdata->baseAddress) |= (uint32_t)interrupt; 
}

/*!
  \brief      disable the TIMER interrupt
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  interrupt: timer interrupt source disable 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_INT_UP: update interrupt disable, TIMERx(x=0..13)
    \arg        TIMER_INT_CH0: channel 0 interrupt disable, TIMERx(x=0..4,7..13)
    \arg        TIMER_INT_CH1: channel 1 interrupt disable, TIMERx(x=0..4,7,8,11)
    \arg        TIMER_INT_CH2: channel 2 interrupt disable, TIMERx(x=0..4,7)
    \arg        TIMER_INT_CH3: channel 3 interrupt disable , TIMERx(x=0..4,7)
    \arg        TIMER_INT_CMT: commutation interrupt disable, TIMERx(x=0,7)
    \arg        TIMER_INT_TRG: trigger interrupt disable, TIMERx(x=0..4,7,8,11)
    \arg        TIMER_INT_BRK: break interrupt disable, TIMERx(x=0,7)
  \retval     none
*/
void timer_interrupt_disable(timer_peripheral_t *timer_pdata, uint32_t interrupt)
{
  TIMER_DMAINTEN(timer_pdata->baseAddress) &= (~(uint32_t)interrupt); 
}

/*!
  \brief      get timer interrupt flag
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  interrupt: the timer interrupt bits
        only one parameter can be selected which is shown as below:
    \arg        TIMER_INT_FLAG_UP: update interrupt flag,TIMERx(x=0..13)
    \arg        TIMER_INT_FLAG_CH0: channel 0 interrupt flag,TIMERx(x=0..4,7..13)
    \arg        TIMER_INT_FLAG_CH1: channel 1 interrupt flag,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_INT_FLAG_CH2: channel 2 interrupt flag,TIMERx(x=0..4,7)
    \arg        TIMER_INT_FLAG_CH3: channel 3 interrupt flag,TIMERx(x=0..4,7)
    \arg        TIMER_INT_FLAG_CMT: channel commutation interrupt flag,TIMERx(x=0,7) 
    \arg        TIMER_INT_FLAG_TRG: trigger interrupt flag,TIMERx(x=0,7,8,11)
    \arg        TIMER_INT_FLAG_BRK:  break interrupt flag,TIMERx(x=0,7)
  \retval     FlagStatus: SET or RESET
*/
FlagStatus timer_interrupt_flag_get(timer_peripheral_t *timer_pdata, uint32_t interrupt)
{
  if (((TIMER_INTF(timer_pdata->baseAddress) & interrupt) != RESET) && ((TIMER_DMAINTEN(timer_pdata->baseAddress) & interrupt) != RESET)) {
    return SET;
  } else {
    return RESET;
  }
}

/*!
  \brief      clear TIMER interrupt flag
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  interrupt: the timer interrupt bits
        only one parameter can be selected which is shown as below:
    \arg        TIMER_INT_FLAG_UP: update interrupt flag,TIMERx(x=0..13)
    \arg        TIMER_INT_FLAG_CH0: channel 0 interrupt flag,TIMERx(x=0..4,7..13)
    \arg        TIMER_INT_FLAG_CH1: channel 1 interrupt flag,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_INT_FLAG_CH2: channel 2 interrupt flag,TIMERx(x=0..4,7)
    \arg        TIMER_INT_FLAG_CH3: channel 3 interrupt flag,TIMERx(x=0..4,7)
    \arg        TIMER_INT_FLAG_CMT: channel commutation interrupt flag,TIMERx(x=0,7) 
    \arg        TIMER_INT_FLAG_TRG: trigger interrupt flag,TIMERx(x=0,7,8,11)
    \arg        TIMER_INT_FLAG_BRK:  break interrupt flag,TIMERx(x=0,7)
  \retval     none
*/
void timer_interrupt_flag_clear(timer_peripheral_t *timer_pdata, uint32_t interrupt)
{
  TIMER_INTF(timer_pdata->baseAddress) = (~(uint32_t)interrupt);
}

/*!
  \brief      get TIMER flags
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  flag: the timer interrupt flags
        only one parameter can be selected which is shown as below:
    \arg        TIMER_FLAG_UP: update flag,TIMERx(x=0..13)
    \arg        TIMER_FLAG_CH0: channel 0 flag,TIMERx(x=0..4,7..13)
    \arg        TIMER_FLAG_CH1: channel 1 flag,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_FLAG_CH2: channel 2 flag,TIMERx(x=0..4,7)
    \arg        TIMER_FLAG_CH3: channel 3 flag,TIMERx(x=0..4,7)
    \arg        TIMER_FLAG_CMT: channel control update flag,TIMERx(x=0,7) 
    \arg        TIMER_FLAG_TRG: trigger flag,TIMERx(x=0,7,8,11) 
    \arg        TIMER_FLAG_BRK: break flag,TIMERx(x=0,7)
    \arg        TIMER_FLAG_CH0O: channel 0 overcapture flag,TIMERx(x=0..4,7..11)
    \arg        TIMER_FLAG_CH1O: channel 1 overcapture flag,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_FLAG_CH2O: channel 2 overcapture flag,TIMERx(x=0..4,7)
    \arg        TIMER_FLAG_CH3O: channel 3 overcapture flag,TIMERx(x=0..4,7)
  \retval     FlagStatus: SET or RESET
*/
FlagStatus timer_flag_get(timer_peripheral_t *timer_pdata, uint32_t flag)
{
  if ((TIMER_INTF(timer_pdata->baseAddress) & flag) != RESET) {
    return SET;
  } else {
    return RESET;
  }
}

/*!
  \brief      clear TIMER flags
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  flag: the timer interrupt flags
        only one parameter can be selected which is shown as below:
    \arg        TIMER_FLAG_UP: update flag,TIMERx(x=0..13)
    \arg        TIMER_FLAG_CH0: channel 0 flag,TIMERx(x=0..4,7..13)
    \arg        TIMER_FLAG_CH1: channel 1 flag,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_FLAG_CH2: channel 2 flag,TIMERx(x=0..4,7)
    \arg        TIMER_FLAG_CH3: channel 3 flag,TIMERx(x=0..4,7)
    \arg        TIMER_FLAG_CMT: channel control update flag,TIMERx(x=0,7) 
    \arg        TIMER_FLAG_TRG: trigger flag,TIMERx(x=0,7,8,11) 
    \arg        TIMER_FLAG_BRK: break flag,TIMERx(x=0,7)
    \arg        TIMER_FLAG_CH0O: channel 0 overcapture flag,TIMERx(x=0..4,7..11)
    \arg        TIMER_FLAG_CH1O: channel 1 overcapture flag,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_FLAG_CH2O: channel 2 overcapture flag,TIMERx(x=0..4,7)
    \arg        TIMER_FLAG_CH3O: channel 3 overcapture flag,TIMERx(x=0..4,7)
  \retval     none
*/
void timer_flag_clear(timer_peripheral_t *timer_pdata, uint32_t flag)
{
  TIMER_INTF(timer_pdata->baseAddress) = (~(uint32_t)flag);
}

/*!
  \brief      enable the TIMER DMA
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  dma: specify which DMA to enable
        only one parameter can be selected which is shown as below:
    \arg        TIMER_DMA_UPD:  update DMA enable,TIMERx(x=0..7)
    \arg        TIMER_DMA_CH0D: channel 0 DMA enable,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CH1D: channel 1 DMA enable,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CH2D: channel 2 DMA enable,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CH3D: channel 3 DMA enable,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CMTD: commutation DMA request enable,TIMERx(x=0,7)
    \arg        TIMER_DMA_TRGD: trigger DMA enable,TIMERx(x=0..4,7)
  \retval     none
*/
void timer_dma_enable(timer_peripheral_t *timer_pdata, uint16_t dma)
{
  TIMER_DMAINTEN(timer_pdata->baseAddress) |= (uint32_t)dma; 
}

/*!
  \brief      disable the TIMER DMA
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  dma: specify which DMA to enable
        one or more parameters can be selected which are shown as below:
    \arg        TIMER_DMA_UPD:  update DMA ,TIMERx(x=0..7)
    \arg        TIMER_DMA_CH0D: channel 0 DMA request,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CH1D: channel 1 DMA request,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CH2D: channel 2 DMA request,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CH3D: channel 3 DMA request,TIMERx(x=0..4,7)
    \arg        TIMER_DMA_CMTD: commutation DMA request ,TIMERx(x=0,7)
    \arg        TIMER_DMA_TRGD: trigger DMA request,TIMERx(x=0..4,7)
  \retval     none
*/
void timer_dma_disable(timer_peripheral_t *timer_pdata, uint16_t dma)
{
  TIMER_DMAINTEN(timer_pdata->baseAddress) &= (~(uint32_t)(dma)); 
}

/*!
  \brief      channel DMA request source selection
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_channel_dma_request_source_channel_event(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL1_DMAS;
}

/*!
  \brief      channel DMA request source selection
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_channel_dma_request_source_update_event(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL1_DMAS;
}

/*!
  \brief      configure the TIMER DMA transfer
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  dma_baseaddr:
        only one parameter can be selected which is shown as below:
     \arg        TIMER_DMACFG_DMATA_CTL0: DMA transfer address is TIMER_CTL0,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CTL1: DMA transfer address is TIMER_CTL1,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_SMCFG: DMA transfer address is TIMER_SMCFG,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_DMAINTEN: DMA transfer address is TIMER_DMAINTEN,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_INTF: DMA transfer address is TIMER_INTF,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_SWEVG: DMA transfer address is TIMER_SWEVG,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CHCTL0: DMA transfer address is TIMER_CHCTL0,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CHCTL1: DMA transfer address is TIMER_CHCTL1,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CHCTL2: DMA transfer address is TIMER_CHCTL2,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CNT: DMA transfer address is TIMER_CNT,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_PSC: DMA transfer address is TIMER_PSC,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CAR: DMA transfer address is TIMER_CAR,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CREP: DMA transfer address is TIMER_CREP,TIMERx(x=0,7)
     \arg        TIMER_DMACFG_DMATA_CH0CV: DMA transfer address is TIMER_CH0CV,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CH1CV: DMA transfer address is TIMER_CH1CV,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CH2CV: DMA transfer address is TIMER_CH2CV,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CH3CV: DMA transfer address is TIMER_CH3CV,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_CCHP: DMA transfer address is TIMER_CCHP,TIMERx(x=0,7)
     \arg        TIMER_DMACFG_DMATA_DMACFG: DMA transfer address is TIMER_DMACFG,TIMERx(x=0..4,7)
     \arg        TIMER_DMACFG_DMATA_DMATB: DMA transfer address is TIMER_DMATB,TIMERx(x=0..4,7)
  \param[in]  dma_lenth:
        only one parameter can be selected which is shown as below:
     \arg        TIMER_DMACFG_DMATC_xTRANSFER(x=1..18): DMA transfer x time
  \retval     none
*/
void timer_dma_transfer_config(timer_peripheral_t *timer_pdata, uint32_t dma_baseaddr, uint32_t dma_length)
{
  TIMER_DMACFG(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_DMACFG_DMATA | TIMER_DMACFG_DMATC));
  TIMER_DMACFG(timer_pdata->baseAddress) |= (uint32_t)(dma_baseaddr | dma_lenth);
}

/*!
  \brief      software generate events 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  event: the timer software event generation sources
        one or more parameters can be selected which are shown as below:
    \arg        TIMER_EVENT_SRC_UPG: update event,TIMERx(x=0..13)
    \arg        TIMER_EVENT_SRC_CH0G: channel 0 capture or compare event generation,TIMERx(x=0..4,7..13) 
    \arg        TIMER_EVENT_SRC_CH1G: channel 1 capture or compare event generation,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_EVENT_SRC_CH2G: channel 2 capture or compare event generation,TIMERx(x=0..4,7) 
    \arg        TIMER_EVENT_SRC_CH3G: channel 3 capture or compare event generation,TIMERx(x=0..4,7) 
    \arg        TIMER_EVENT_SRC_CMTG: channel commutation event generation,TIMERx(x=0,7) 
    \arg        TIMER_EVENT_SRC_TRGG: trigger event generation,TIMERx(x=0..4,7,8,11)
    \arg        TIMER_EVENT_SRC_BRKG:  break event generation,TIMERx(x=0,7)
  \retval     none
*/
void timer_event_software_generate(timer_peripheral_t *timer_pdata, uint16_t event)
{
  TIMER_SWEVG(timer_pdata->baseAddress) |= (uint32_t)event;
}

/*!
  \brief      initialize TIMER break parameter struct with a default value
  \param[in]  breakpara: TIMER break parameter struct
  \retval     none
*/
void timer_break_config_init(timer_peripheral_t *timer_pdata)
{
  /* initialize the break parameter struct member with the default value */
  timer_pdata->timer_break_config->run_off_state = TIMER_ROS_STATE_DISABLE;
  timer_pdata->timer_break_config->idle_off_state = TIMER_IOS_STATE_DISABLE;
  timer_pdata->timer_break_config->dead_time = 0U;
  timer_pdata->timer_break_config->break_polarity = TIMER_BREAK_POLARITY_LOW;
  timer_pdata->timer_break_config->output_auto_state = TIMER_OUTAUTO_DISABLE;
  timer_pdata->timer_break_config->protect_mode = TIMER_CCHP_PROT_OFF;
  timer_pdata->timer_break_config->break_state = TIMER_BREAK_DISABLE;
}

/*!
  \brief      configure TIMER break function 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  breakpara: TIMER break parameter struct
        runoffstate: TIMER_ROS_STATE_ENABLE,TIMER_ROS_STATE_DISABLE
        ideloffstate: TIMER_IOS_STATE_ENABLE,TIMER_IOS_STATE_DISABLE
        deadtime: 0~255
        breakpolarity: TIMER_BREAK_POLARITY_LOW,TIMER_BREAK_POLARITY_HIGH
        outputautostate: TIMER_OUTAUTO_ENABLE,TIMER_OUTAUTO_DISABLE
        protectmode: TIMER_CCHP_PROT_OFF,TIMER_CCHP_PROT_0,TIMER_CCHP_PROT_1,TIMER_CCHP_PROT_2
        breakstate: TIMER_BREAK_ENABLE,TIMER_BREAK_DISABLE
  \retval     none
*/
void timer_break_config(timer_peripheral_t *timer_pdata)
{
  TIMER_CCHP(timer_pdata->baseAddress) = (uint32_t)(((uint32_t)(timer_pdata->timer_break_config->run_off_state)) |
                      ((uint32_t)(timer_pdata->timer_break_config->idle_off_state)) |
                      ((uint32_t)(timer_pdata->timer_break_config->dead_time)) |
                      ((uint32_t)(timer_pdata->timer_break_config->break_polarity)) |
                      ((uint32_t)(timer_pdata->timer_break_config->output_auto_state)) |
                      ((uint32_t)(timer_pdata->timer_break_config->protect_mode)) |
                      ((uint32_t)(timer_pdata->timer_break_config->break_state)));
}

/*!
  \brief      enable TIMER break function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_break_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CCHP(timer_pdata->baseAddress) |= (uint32_t)TIMER_CCHP_BRKEN;
}

/*!
  \brief      disable TIMER break function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_break_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CCHP(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CCHP_BRKEN;
}

/*!
  \brief      enable TIMER output automatic function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_automatic_output_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CCHP(timer_pdata->baseAddress) |= (uint32_t)TIMER_CCHP_OAEN;
}

/*!
  \brief      disable TIMER output automatic function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_automatic_output_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CCHP(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CCHP_OAEN;
}

/*!
  \brief      enable TIMER primary output function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_primary_output_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CCHP(timer_pdata->baseAddress) |= (uint32_t)TIMER_CCHP_POEN;
}

/*!
  \brief      disable TIMER primary output function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_primary_output_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CCHP(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CCHP_POEN);
}

/*!
  \brief      enable channel capture/compare control shadow register
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_channel_control_shadow_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL1_CCSE;
}

/*!
  \brief      disable channel capture/compare control shadow register
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_channel_control_shadow_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_CCSE);
}

/*!
  \brief      configure TIMER channel control shadow register update when CMTG bit is set
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_channel_control_shadow_update_ccu(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_CCUC);
}

/*!
  \brief      configure TIMER channel control shadow register update when CMTG bit is set or an rising edge of TRGI occurs
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_channel_control_shadow_update_ccutri(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL1_CCUC;
}


/*!
  \brief      initialize TIMER channel output config with default values
  \param[in]  ocpara: TIMER channel n output parameter struct
  \retval     none
*/
void timer_channel_output_config_init(timer_peripheral_t *timer_pdata)
{
  /* initialize the channel output parameter struct member with the default value */
  timer_pdata->channel_output_config->output_state  = (uint16_t)TIMER_CCX_DISABLE;
  timer_pdata->channel_output_config->output_nstate = TIMER_CCXN_DISABLE;
  timer_pdata->channel_output_config->oc_polarity   = TIMER_OC_POLARITY_HIGH;
  timer_pdata->channel_output_config->oc_npolarity  = TIMER_OCN_POLARITY_HIGH;
  timer_pdata->channel_output_config->oc_idle_state  = TIMER_OC_IDLE_STATE_LOW;
  timer_pdata->channel_output_config->oc_nidle_state = TIMER_OCN_IDLE_STATE_LOW;
}

/*!
  \brief      configure TIMER channel output function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel 0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel 1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel 2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel 3(TIMERx(x=0..4,7))
        outputstate: TIMER_CCX_ENABLE,TIMER_CCX_DISABLE
        outputnstate: TIMER_CCXN_ENABLE,TIMER_CCXN_DISABLE
        ocpolarity: TIMER_OC_POLARITY_HIGH,TIMER_OC_POLARITY_LOW
        ocnpolarity: TIMER_OCN_POLARITY_HIGH,TIMER_OCN_POLARITY_LOW
        ocidlestate: TIMER_OC_IDLE_STATE_LOW,TIMER_OC_IDLE_STATE_HIGH
        ocnidlestate: TIMER_OCN_IDLE_STATE_LOW,TIMER_OCN_IDLE_STATE_HIGH
  \retval     none
*/
void timer_channel_output_config(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
    case TIMER_CH_0:
      /* reset the CH0EN bit */
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
      TIMER_CHCTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CHCTL0_CH0MS;
      /* set the CH0EN bit */
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_output_config->output_state;
      /* reset the CH0P bit */
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0P);
      /* set the CH0P bit */
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_output_config->oc_polarity;
      if ((timer_pdata->baseAddress == TIMER0_BASE) || (timer_pdata->baseAddress == TIMER7_BASE)) {
        TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0NEN);
        TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_output_config->output_nstate;
        TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
        TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_output_config->oc_npolarity;
        TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_ISO0);
        TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_output_config->oc_idle_state;
        TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_ISO0N);
        TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_output_config->oc_nidle_state;
      }
      break;
    case TIMER_CH_1:
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
      TIMER_CHCTL0(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CHCTL0_CH1MS;
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)timer_pdata->channel_output_config->output_state << 4U);
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1P);
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_polarity) << 4U);
      if ((timer_pdata->baseAddress == TIMER0_BASE) || (timer_pdata->baseAddress == TIMER7_BASE)) {
        TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1NEN);
        TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(channel_output_config->output_nstate) << 4U);
        TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
        TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(channel_output_config->oc_npolarity) << 4U);
        TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_ISO1);
        TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_idle_state) << 2U);
        TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_ISO1N);
        TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_nidle_state) << 2U);
      }
      break;
    case TIMER_CH_2:
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
      TIMER_CHCTL1(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CHCTL1_CH2MS;
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)timer_pdata->channel_output_config->output_state << 8U);
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2P);
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_polarity) << 8U);
      if ((timer_pdata->baseAddress == TIMER0_BASE) || (timer_pdata->baseAddress == TIMER7_BASE)) {
        TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2NEN);
        TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->output_nstate) << 8U);
        TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
        TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_npolarity) << 8U);
        TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_ISO2);
        TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_idle_state) << 4U);
        TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_ISO2N);
        TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_nidle_state) << 4U);
      }
      break;
    case TIMER_CH_3:
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
      TIMER_CHCTL1(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CHCTL1_CH3MS;
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)timer_pdata->channel_output_config->output_state << 12U);
      TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH3P);
      TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_polarity) << 12U);
      if ((timer_pdata->baseAddress == TIMER0_BASE) || (timer_pdata->baseAddress == TIMER7_BASE)) {
        TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_ISO3);
        TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_output_config->oc_idle_state) << 6U);
      }
      break;
    default:
      break;
  }
}

/*!
  \brief      configure TIMER channel output compare mode
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \arg        TIMER_OC_MODE_TIMING: timing mode
    \arg        TIMER_OC_MODE_ACTIVE: active mode
    \arg        TIMER_OC_MODE_INACTIVE: inactive mode
    \arg        TIMER_OC_MODE_TOGGLE: toggle mode
    \arg        TIMER_OC_MODE_LOW: force low mode
    \arg        TIMER_OC_MODE_HIGH: force high mode
    \arg        TIMER_OC_MODE_PWM0: PWM0 mode
    \arg        TIMER_OC_MODE_PWM1: PWM1 mode
  \retval     none
*/
void timer_channel_output_mode_config(timer_peripheral_t *timer_pdata)
{
  switch(timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0COMCTL);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->outputCompareMode;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1COMCTL);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->outputCompareMode) << 8U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2COMCTL);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->outputCompareMode;
    break;
  case TIMER_CH_3:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3COMCTL);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->outputCompareMode) << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      configure TIMER channel output pulse value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \param[in]  pulse: channel output pulse value 0~65535
  \retval     none
*/
void timer_channel_output_pulse_value_config(timer_peripheral_t *timer_pdata, uint32_t pulse)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CH0CV(timer_pdata->baseAddress) = (uint32_t)pulse;
    break;
  case TIMER_CH_1:
    TIMER_CH1CV(timer_pdata->baseAddress) = (uint32_t)pulse;
    break;
  case TIMER_CH_2:
    TIMER_CH2CV(timer_pdata->baseAddress) = (uint32_t)pulse;
    break;
  case TIMER_CH_3:
     TIMER_CH3CV(timer_pdata->baseAddress) = (uint32_t)pulse;
    break;
  default:
    break;
  }
}

/*!
  \brief      enable TIMER channel output shadow function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \retval     none
*/
void timer_channel_output_shadow_enable(timer_peripheral_t *timer_pdata)
{
  switch(timer_pdata->timer_channel){
  case TIMER_CH_0:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0COMSEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_SHADOW_ENABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1COMSEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(TIMER_OC_SHADOW_ENABLE) << 8U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2COMSEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_SHADOW_ENABLE;
    break;
  case TIMER_CH_3:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3COMSEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(TIMER_OC_SHADOW_ENABLE) << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      disable TIMER channel output shadow function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \retval     none
*/
void timer_channel_output_shadow_disable(timer_peripheral_t *timer_pdata)
{
  switch(timer_pdata->timer_channel){
  case TIMER_CH_0:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0COMSEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_SHADOW_DISABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1COMSEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(TIMER_OC_SHADOW_DISABLE) << 8U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2COMSEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_SHADOW_DISABLE;
    break;
  case TIMER_CH_3:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3COMSEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(TIMER_OC_SHADOW_DISABLE) << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      enable TIMER channel output fast function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  channel:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \param[in]  ocfast: channel output fast function
        only one parameter can be selected which is shown as below:
    \arg        TIMER_OC_FAST_ENABLE: channel output fast function enable
    \arg        TIMER_OC_FAST_DISABLE: channel output fast function disable
  \retval     none
*/
void timer_channel_output_fast_enable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0COMFEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_FAST_ENABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1COMFEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_FAST_ENABLE << 8U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2COMFEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_FAST_ENABLE;
    break;
  case TIMER_CH_3:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3COMFEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_FAST_ENABLE << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      disable TIMER channel output fast function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  channel:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \param[in]  ocfast: channel output fast function
        only one parameter can be selected which is shown as below:
    \arg        TIMER_OC_FAST_ENABLE: channel output fast function enable
    \arg        TIMER_OC_FAST_DISABLE: channel output fast function disable
  \retval     none
*/
void timer_channel_output_fast_disable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0COMFEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_FAST_DISABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1COMFEN);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_FAST_DISABLE << 8U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2COMFEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_FAST_DISABLE;
    break;
  case TIMER_CH_3:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3COMFEN);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_FAST_DISABLE << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      enable TIMER channel output clear function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0
    \arg        TIMER_CH_1: TIMER channel1
    \arg        TIMER_CH_2: TIMER channel2
    \arg        TIMER_CH_3: TIMER channel3
  \retval     none
*/
void timer_channel_output_clear_enable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
    case TIMER_CH_0:
      TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0COMCEN);
      TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_CLEAR_ENABLE;
      break;
    case TIMER_CH_1:
      TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1COMCEN);
      TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_CLEAR_ENABLE << 8U);
      break;
    case TIMER_CH_2:
      TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2COMCEN);
      TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_CLEAR_ENABLE;
      break;
    case TIMER_CH_3:
      TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3COMCEN);
      TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_CLEAR_ENABLE << 8U);
      break;
    default:
      break;
  }
}

/*!
  \brief      disable TIMER channel output clear function
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0
    \arg        TIMER_CH_1: TIMER channel1
    \arg        TIMER_CH_2: TIMER channel2
    \arg        TIMER_CH_3: TIMER channel3
  \retval     none
*/
void timer_channel_output_clear_disable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
    case TIMER_CH_0:
      TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0COMCEN);
      TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_CLEAR_DISABLE;
      break;
    case TIMER_CH_1:
      TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1COMCEN);
      TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_CLEAR_DISABLE << 8U);
      break;
    case TIMER_CH_2:
      TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2COMCEN);
      TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_CLEAR_DISABLE;
      break;
    case TIMER_CH_3:
      TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3COMCEN);
      TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_CLEAR_DISABLE << 8U);
      break;
    default:
      break;
  }
}

/*!
  \brief      set TIMER channel output polarity high
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \retval     none
*/
void timer_channel_output_polarity_high(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_POLARITY_HIGH;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_POLARITY_HIGH << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_POLARITY_HIGH << 8U);
    break;
  case TIMER_CH_3:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH3P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_POLARITY_HIGH << 12U);
    break;
  default:
    break;
  }
}

/*!
  \brief      set TIMER channel output polarity low
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \retval     none
*/
void timer_channel_output_polarity_low(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_OC_POLARITY_LOW;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_POLARITY_LOW << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_POLARITY_LOW << 8U);
    break;
  case TIMER_CH_3:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH3P);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OC_POLARITY_LOW << 12U);
    break;
  default:
    break;
  }
}

/*!
  \brief      set TIMER channel complementary output polarity high
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,7))
  \retval     none
*/
void timer_channel_complementary_output_polarity_high(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_OCN_POLARITY_HIGH;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OCN_POLARITY_HIGH << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OCN_POLARITY_HIGH << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      set TIMER channel complementary output polarity low
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,7))
  \retval     none
*/
void timer_channel_complementary_output_polarity_low(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_OCN_POLARITY_LOW;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OCN_POLARITY_LOW << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_OCN_POLARITY_LOW << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      enable TIMER channel state
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \retval     none
*/
void timer_channel_output_state_enable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CCX_ENABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCX_ENABLE << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCX_ENABLE << 8U);
    break;
  case TIMER_CH_3:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCX_ENABLE << 12U);
    break;
  default:
    break;
  }
}

/*!
  \brief      disable TIMER channel state
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \retval     none
*/
void timer_channel_output_state_disable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CCX_DISABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCX_DISABLE << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCX_DISABLE << 8U);
    break;
  case TIMER_CH_3:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCX_DISABLE << 12U);
    break;
  default:
    break;
  }
}

/*!
  \brief      enable TIMER channel complementary output state
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,7))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,7))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,7))
  \retval     none
*/
void timer_channel_complementary_output_state_enable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0NEN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CCXN_ENABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1NEN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCXN_ENABLE << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2NEN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCXN_ENABLE << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      disable TIMER channel complementary output state
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,7))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,7))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,7))
  \retval     none
*/
void timer_channel_complementary_output_state_disable(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0NEN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CCXN_DISABLE;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1NEN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCXN_DISABLE << 4U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2NEN);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_CCXN_DISABLE << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      initialize TIMER channel input config with default values
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_channel_input_config_init(timer_peripheral_t *timer_pdata)
{
  timer_pdata->channel_input_config->icpolarity = TIMER_IC_POLARITY_RISING;
  timer_pdata->channel_input_config->icselection = TIMER_IC_SELECTION_DIRECTTI;
  timer_pdata->channel_input_config->icprescaler = TIMER_IC_PSC_DIV1;
  timer_pdata->channel_input_config->icfilter = 0U;
}

/*!
  \brief      configure TIMER input capture parameter 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
         icpolarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING
         icselection: TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI,TIMER_IC_SELECTION_ITS
         icprescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
         icfilter: 0~15
  \retval      none
*/
void timer_input_capture_config(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)(timer_pdata->channel_input_config->ic_polarity);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)(timer_pdata->channel_input_config->ic_selection);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 4U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH0EN;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_polarity) << 4U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_selection) << 8U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 12U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH1EN;
    break;
  case TIMER_CH_2:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH2P|TIMER_CHCTL2_CH2NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_polarity) << 8U);
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2MS);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_selection));
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2CAPFLT);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 4U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH2EN;
    break;
  case TIMER_CH_3:
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH3EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH3P));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_polarity) << 12U);
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3MS);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_selection) << 8U);
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3CAPFLT);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 12U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH3EN;
    break;
  default:
    break;
  }
  /* configure TIMER channel input capture prescaler value */
  timer_channel_input_capture_prescaler_config(timer_pdata->baseAddress);
}

/*!
  \brief      configure TIMER channel input capture prescaler value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \arg        TIMER_IC_PSC_DIV1: no prescaler
    \arg        TIMER_IC_PSC_DIV2: divided by 2
    \arg        TIMER_IC_PSC_DIV4: divided by 4
    \arg        TIMER_IC_PSC_DIV8: divided by 8
  \retval     none
*/
void timer_channel_input_capture_prescaler_config(timer_peripheral_t *timer_pdata)
{
  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPPSC);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_input_config->ic_prescaler;
    break;
  case TIMER_CH_1:
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPPSC);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= ((uint32_t)timer_pdata->channel_input_config->ic_prescaler << 8U);
    break;
  case TIMER_CH_2:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH2CAPPSC);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= (uint32_t)timer_pdata->channel_input_config->ic_prescaler;
    break;
  case TIMER_CH_3:
    TIMER_CHCTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL1_CH3CAPPSC);
    TIMER_CHCTL1(timer_pdata->baseAddress) |= ((uint32_t)timer_pdata->channel_input_config->ic_prescaler << 8U);
    break;
  default:
    break;
  }
}

/*!
  \brief      read TIMER channel capture compare register value
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
    \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
    \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
    \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
  \retval     channel capture compare register value
*/
uint32_t timer_channel_capture_value_register_read(timer_peripheral_t *timer_pdata)
{
  uint32_t count_value = 0U;

  switch (timer_pdata->timer_channel) {
  case TIMER_CH_0:
    count_value = TIMER_CH0CV(timer_pdata->baseAddress);
    break;
  case TIMER_CH_1:
    count_value = TIMER_CH1CV(timer_pdata->baseAddress);
    break;
  case TIMER_CH_2:
    count_value = TIMER_CH2CV(timer_pdata->baseAddress);
    break;
  case TIMER_CH_3:
    count_value = TIMER_CH3CV(timer_pdata->baseAddress);
    break;
  default:
    break;
  }
  return (count_value);
}

/*!
  \brief      configure TIMER input pwm capture function 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
    \arg        TIMER_CH_0: TIMER channel0
    \arg        TIMER_CH_1: TIMER channel1
         icpolarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING
         icselection: TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI
         icprescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
         icfilter: 0~15
  \retval     none
*/
void timer_input_pwm_capture_config(timer_peripheral_t *timer_pdata)
{
  uint16_t ic_polarity  = 0x0U;
  uint16_t ic_selection = 0x0U;

  /* Set channel input polarity */
  if (timer_pdata->channel_input_config->ic_polarity == TIMER_IC_POLARITY_RISING) {
    ic_polarity = TIMER_IC_POLARITY_FALLING;
  } else {
    ic_polarity = TIMER_IC_POLARITY_RISING;
  }

  /* Set channel input pwm parameter */
  if (timer_pdata->channel_input_config->ic_selection == TIMER_IC_SELECTION_DIRECTTI) {
    ic_selection = TIMER_IC_SELECTION_INDIRECTTI;
  } else {
    ic_selection = TIMER_IC_SELECTION_DIRECTTI;
  }

  if (timer_pdata->timer_channel == TIMER_CH_0) {
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)(timer_pdata->channel_input_config->ic_polarity);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)(timer_pdata->channel_input_config->ic_selection);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= ((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 4U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH0EN;
    /* configure TIMER channel input capture prescaler value */
    timer_channel_input_capture_prescaler_config(timer_pdata->baseAddress);

    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)ic_polarity << 4U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)ic_selection << 8U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 12U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH1EN;
    /* configure TIMER channel input capture prescaler value */
    timer_channel_input_capture_prescaler_config(timer_pdata->baseAddress);
  } else {
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_polarity) << 4U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_selection) << 8U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 12U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH1EN;
    /* configure TIMER channel input capture prescaler value */
    timer_channel_input_capture_prescaler_config(timer_pdata->baseAddress);

    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH0P|TIMER_CHCTL2_CH0NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)ic_polarity;
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)ic_selection;
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= ((uint32_t)(timer_pdata->channel_input_config->ic_filter) << 4U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH0EN;
    /* configure TIMER channel input capture prescaler value */
    timer_channel_input_capture_prescaler_config(timer_pdata->baseAddress);
  }
}

/*!
  \brief      enable TIMER hall sensor
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_hall_mode_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)TIMER_CTL1_TI0S;
}

/*!
  \brief      disable TIMER hall sensor
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_hall_mode_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CTL1(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CTL1_TI0S;
}

/*!
  \brief      select TIMER input trigger source 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  intrigger:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_SMCFG_TRGSEL_ITI0: internal trigger 0
    \arg        TIMER_SMCFG_TRGSEL_ITI1: internal trigger 1
    \arg        TIMER_SMCFG_TRGSEL_ITI2: internal trigger 2
    \arg        TIMER_SMCFG_TRGSEL_ITI3: internal trigger 3
    \arg        TIMER_SMCFG_TRGSEL_CI0F_ED: TI0 edge detector
    \arg        TIMER_SMCFG_TRGSEL_CI0FE0: filtered TIMER input 0
    \arg        TIMER_SMCFG_TRGSEL_CI1FE1: filtered TIMER input 1
    \arg        TIMER_SMCFG_TRGSEL_ETIFP: external trigger(x=0..4,7)
  \retval     none
*/
void timer_input_trigger_source_select(timer_peripheral_t *timer_pdata, uint32_t input_trigger)
{
  TIMER_SMCFG(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_SMCFG_TRGS);
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)input_trigger;
}

/*!
  \brief      select TIMER master mode output trigger source 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  outrigger: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_TRI_OUT_SRC_RESET: the UPG bit as trigger output
    \arg        TIMER_TRI_OUT_SRC_ENABLE: the counter enable signal TIMER_CTL0_CEN as trigger output
    \arg        TIMER_TRI_OUT_SRC_UPDATE: update event as trigger output
    \arg        TIMER_TRI_OUT_SRC_CH0: a capture or a compare match occurred in channal0 as trigger output TRGO
    \arg        TIMER_TRI_OUT_SRC_O0CPRE: O0CPRE as trigger output
    \arg        TIMER_TRI_OUT_SRC_O1CPRE: O1CPRE as trigger output
    \arg        TIMER_TRI_OUT_SRC_O2CPRE: O2CPRE as trigger output
    \arg        TIMER_TRI_OUT_SRC_O3CPRE: O3CPRE as trigger output
  \retval     none
*/
void timer_master_output_trigger_source_select(timer_peripheral_t *timer_pdata, uint32_t output_trigger)
{
  TIMER_CTL1(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CTL1_MMC);
  TIMER_CTL1(timer_pdata->baseAddress) |= (uint32_t)output_trigger;
}

/*!
  \brief      select TIMER slave mode 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  slavemode:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_SLAVE_MODE_DISABLE: slave mode disable
    \arg        TIMER_ENCODER_MODE0: encoder mode 0
    \arg        TIMER_ENCODER_MODE1: encoder mode 1
    \arg        TIMER_ENCODER_MODE2: encoder mode 2
    \arg        TIMER_SLAVE_MODE_RESTART: restart mode
    \arg        TIMER_SLAVE_MODE_PAUSE: pause mode
    \arg        TIMER_SLAVE_MODE_EVENT: event mode
    \arg        TIMER_SLAVE_MODE_EXTERNAL0: external clock mode 0.
  \retval     none
*/

void timer_slave_mode_select(timer_peripheral_t *timer_pdata, uint32_t slave_mode)
{
  TIMER_SMCFG(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_SMCFG_SMC);
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)slave_mode;
}

/*!
  \brief      enable TIMER master slave mode 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/ 
void timer_master_slave_mode_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)TIMER_SMCFG_MSM;
}

/*!
  \brief      disable TIMER master slave mode 
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/ 
void timer_master_slave_mode_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_SMCFG(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_SMCFG_MSM;
}

/*!
  \brief      configure TIMER external trigger input
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  extprescaler:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_EXT_TRI_PSC_OFF: no divided
    \arg        TIMER_EXT_TRI_PSC_DIV2: divided by 2
    \arg        TIMER_EXT_TRI_PSC_DIV4: divided by 4
    \arg        TIMER_EXT_TRI_PSC_DIV8: divided by 8
  \param[in]  extpolarity:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_ETP_FALLING: active low or falling edge active
    \arg        TIMER_ETP_RISING: active high or rising edge active
  \param[in]  extfilter: a value between 0 and 15
  \retval     none
*/
void timer_external_trigger_config(timer_peripheral_t *timer_pdata, uint32_t extprescaler,
                   uint32_t extpolarity, uint32_t extfilter)
{
  TIMER_SMCFG(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_SMCFG_ETP | TIMER_SMCFG_ETPSC | TIMER_SMCFG_ETFC));
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)(extprescaler | extpolarity);
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)(extfilter << 8U);
}

/*!
  \brief      configure TIMER quadrature decoder mode
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  decomode: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_ENCODER_MODE0: counter counts on CI0FE0 edge depending on CI1FE1 level
    \arg        TIMER_ENCODER_MODE1: counter counts on CI1FE1 edge depending on CI0FE0 level
    \arg        TIMER_ENCODER_MODE2: counter counts on both CI0FE0 and CI1FE1 edges depending on the level of the other input
  \param[in]  ic0polarity: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_IC_POLARITY_RISING: capture rising edge
    \arg        TIMER_IC_POLARITY_FALLING: capture falling edge
  \param[in]  ic1polarity:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_IC_POLARITY_RISING: capture rising edge
    \arg        TIMER_IC_POLARITY_FALLING: capture falling edge
  \retval     none
*/
void timer_quadrature_decoder_mode_config(timer_peripheral_t *timer_pdata, uint32_t decomode,
                   uint16_t ic0polarity, uint16_t ic1polarity)
{
  TIMER_SMCFG(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_SMCFG_SMC);
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)decomode;

  TIMER_CHCTL0(timer_pdata->baseAddress) &= (uint32_t)(((~(uint32_t)TIMER_CHCTL0_CH0MS)) & ((~(uint32_t)TIMER_CHCTL0_CH1MS)));
  TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)(TIMER_IC_SELECTION_DIRECTTI | ((uint32_t)TIMER_IC_SELECTION_DIRECTTI << 8U));

  TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
  TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
  TIMER_CHCTL2(timer_pdata->baseAddress) |= ((uint32_t)ic0polarity | ((uint32_t)ic1polarity << 4U));
}

/*!
  \brief      configure TIMER internal clock mode
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_internal_clock_config(timer_peripheral_t *timer_pdata)
{
  TIMER_SMCFG(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_SMCFG_SMC;
}

/*!
  \brief      configure TIMER the internal trigger as external clock input
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  intrigger:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_SMCFG_TRGSEL_ITI0: internal trigger 0
    \arg        TIMER_SMCFG_TRGSEL_ITI1: internal trigger 1
    \arg        TIMER_SMCFG_TRGSEL_ITI2: internal trigger 2
    \arg        TIMER_SMCFG_TRGSEL_ITI3: internal trigger 3
  \retval     none
*/
void timer_internal_trigger_as_external_clock_config(timer_peripheral_t *timer_pdata, uint32_t input_trigger)
{
  timer_input_trigger_source_select(timer_periph, input_trigger);
  TIMER_SMCFG(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_SMCFG_SMC;
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
}

/*!
  \brief      configure TIMER the external trigger as external clock input
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  extrigger:
        only one parameter can be selected which is shown as below:
    \arg        TIMER_SMCFG_TRGSEL_CI0F_ED: TI0 edge detector
    \arg        TIMER_SMCFG_TRGSEL_CI0FE0: filtered TIMER input 0
    \arg        TIMER_SMCFG_TRGSEL_CI1FE1: filtered TIMER input 1
  \param[in]  extpolarity: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_IC_POLARITY_RISING: active high or rising edge active
    \arg        TIMER_IC_POLARITY_FALLING: active low or falling edge active
  \param[in]  extfilter: a value between 0 and 15
  \retval     none
*/
void timer_external_trigger_as_external_clock_config(timer_peripheral_t *timer_pdata, uint32_t extrigger,
                     uint16_t extpolarity, uint32_t extfilter)
{
  if (TIMER_SMCFG_TRGSEL_CI1FE1 == extrigger) {
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH1P|TIMER_CHCTL2_CH1NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)extpolarity << 4U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)((uint32_t)TIMER_IC_SELECTION_DIRECTTI << 8U);
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)(extfilter << 12U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH1EN;
  } else {
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
    TIMER_CHCTL2(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_CHCTL2_CH0P|TIMER_CHCTL2_CH0NP));
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)extpolarity;
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)TIMER_IC_SELECTION_DIRECTTI;
    TIMER_CHCTL0(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
    TIMER_CHCTL0(timer_pdata->baseAddress) |= (uint32_t)(extfilter << 4U);
    TIMER_CHCTL2(timer_pdata->baseAddress) |= (uint32_t)TIMER_CHCTL2_CH0EN;
  }
  timer_input_trigger_source_select(timer_periph, extrigger);
  TIMER_SMCFG(timer_pdata->baseAddress) &= (~(uint32_t)TIMER_SMCFG_SMC);
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)TIMER_SLAVE_MODE_EXTERNAL0;
}

/*!
  \brief      configure TIMER the external clock mode0
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  extprescaler: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_EXT_TRI_PSC_OFF: no divided
    \arg        TIMER_EXT_TRI_PSC_DIV2: divided by 2
    \arg        TIMER_EXT_TRI_PSC_DIV4: divided by 4
    \arg        TIMER_EXT_TRI_PSC_DIV8: divided by 8
  \param[in]  extpolarity: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_ETP_FALLING: active low or falling edge active
    \arg        TIMER_ETP_RISING: active high or rising edge active
  \param[in]  extfilter: a value between 0 and 15
  \retval     none
*/
void timer_external_clock_mode0_config(timer_peripheral_t *timer_pdata, uint32_t extprescaler,
                     uint32_t extpolarity, uint32_t extfilter)
{
  timer_external_trigger_config(timer_periph, extprescaler, extpolarity, extfilter);
  TIMER_SMCFG(timer_pdata->baseAddress) &= (~(uint32_t)(TIMER_SMCFG_SMC | TIMER_SMCFG_TRGS));
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)(TIMER_SLAVE_MODE_EXTERNAL0 | TIMER_SMCFG_TRGSEL_ETIFP);
}

/*!
  \brief      configure TIMER the external clock mode1
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \param[in]  extprescaler: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_EXT_TRI_PSC_OFF: no divided
    \arg        TIMER_EXT_TRI_PSC_DIV2: divided by 2
    \arg        TIMER_EXT_TRI_PSC_DIV4: divided by 4
    \arg        TIMER_EXT_TRI_PSC_DIV8: divided by 8
  \param[in]  extpolarity: 
        only one parameter can be selected which is shown as below:
    \arg        TIMER_ETP_FALLING: active low or falling edge active
    \arg        TIMER_ETP_RISING: active high or rising edge active
  \param[in]  extfilter: a value between 0 and 15
  \retval     none
*/
void timer_external_clock_mode1_config(timer_peripheral_t *timer_pdata, uint32_t extprescaler,
                     uint32_t extpolarity, uint32_t extfilter)
{
  timer_external_trigger_config(timer_periph, extprescaler, extpolarity, extfilter);
  TIMER_SMCFG(timer_pdata->baseAddress) |= (uint32_t)TIMER_SMCFG_SMC1;
}

/*!
  \brief      disable TIMER the external clock mode1
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_external_clock_mode1_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_SMCFG(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_SMCFG_SMC1;
}

/*!
  \brief      enable TIMER write CHxVAL register selection
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_write_chxval_register_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CFG(timer_pdata->baseAddress) |= (uint32_t)TIMER_CFG_CHVSEL;
}

/*!
  \brief      disable TIMER write CHxVAL register selection
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_write_chxval_register_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CFG(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CFG_CHVSEL;
}

/*!
  \brief      enable TIMER output value selection
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_output_value_selection_enable(timer_peripheral_t *timer_pdata)
{
  TIMER_CFG(timer_pdata->baseAddress) |= (uint32_t)TIMER_CFG_OUTSEL;
}

/*!
  \brief      disable TIMER output value selection
  \param[in]  timer_pdata: pointer to timer_peripheral_t structure
  \retval     none
*/
void timer_output_value_selection_disable(timer_peripheral_t *timer_pdata)
{
  TIMER_CFG(timer_pdata->baseAddress) &= ~(uint32_t)TIMER_CFG_OUTSEL;
}

