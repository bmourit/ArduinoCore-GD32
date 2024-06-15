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
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     none
*/
void timer_deinit(uint32_t timer_periph)
{
    switch(timer_periph) {
    case TIMER0:
        /* reset TIMER0 */
        rcu_periph_reset_enable(RCU_TIMER0RST);
        rcu_periph_reset_disable(RCU_TIMER0RST);
        break;
    case TIMER1:
        /* reset TIMER1 */
        rcu_periph_reset_enable(RCU_TIMER1RST);
        rcu_periph_reset_disable(RCU_TIMER1RST);
        break;
    case TIMER2:
        /* reset TIMER2 */
        rcu_periph_reset_enable(RCU_TIMER2RST);
        rcu_periph_reset_disable(RCU_TIMER2RST);
        break;
    case TIMER3:
        /* reset TIMER3 */
        rcu_periph_reset_enable(RCU_TIMER3RST);
        rcu_periph_reset_disable(RCU_TIMER3RST);
        break;
    case TIMER4:
        /* reset TIMER4 */
        rcu_periph_reset_enable(RCU_TIMER4RST);
        rcu_periph_reset_disable(RCU_TIMER4RST);
        break;
    case TIMER5:
        /* reset TIMER5 */
        rcu_periph_reset_enable(RCU_TIMER5RST);
        rcu_periph_reset_disable(RCU_TIMER5RST);
        break;
    case TIMER6:
        /* reset TIMER6 */
        rcu_periph_reset_enable(RCU_TIMER6RST);
        rcu_periph_reset_disable(RCU_TIMER6RST);
        break;
    case TIMER7:
        /* reset TIMER7 */
        rcu_periph_reset_enable(RCU_TIMER7RST);
        rcu_periph_reset_disable(RCU_TIMER7RST);
        break;
#ifndef GD32F30X_HD
    case TIMER8:
        /* reset TIMER8 */
        rcu_periph_reset_enable(RCU_TIMER8RST);
        rcu_periph_reset_disable(RCU_TIMER8RST);
        break;
    case TIMER9:
        /* reset TIMER9 */
        rcu_periph_reset_enable(RCU_TIMER9RST);
        rcu_periph_reset_disable(RCU_TIMER9RST);
        break;
    case TIMER10:
        /* reset TIMER10 */
        rcu_periph_reset_enable(RCU_TIMER10RST);
        rcu_periph_reset_disable(RCU_TIMER10RST);
        break;
    case TIMER11:
        /* reset TIMER11 */
        rcu_periph_reset_enable(RCU_TIMER11RST);
        rcu_periph_reset_disable(RCU_TIMER11RST);
        break;
    case TIMER12:
        /* reset TIMER12 */
        rcu_periph_reset_enable(RCU_TIMER12RST);
        rcu_periph_reset_disable(RCU_TIMER12RST);
        break;
    case TIMER13:
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
    \param[out] initpara: init parameter struct
    \retval     none
*/
void timer_struct_para_init(timer_parameter_struct* initpara)
{
    static const timer_parameter_struct default_initpara = {
        0U, TIMER_COUNTER_EDGE, TIMER_COUNTER_UP, 65535U, TIMER_CKDIV_DIV1, 0U
    };

    *initpara = default_initpara;
}

/*!
    \brief      initialize TIMER counter
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[in]  initpara: init parameter struct
                  prescaler: prescaler value of the counter clock, 0~65535
                  alignedmode: TIMER_COUNTER_EDGE, TIMER_COUNTER_CENTER_DOWN, TIMER_COUNTER_CENTER_UP, TIMER_COUNTER_CENTER_BOTH
                  counterdirection: TIMER_COUNTER_UP, TIMER_COUNTER_DOWN
                  period: counter auto reload value, 0~65535
                  clockdivision: TIMER_CKDIV_DIV1, TIMER_CKDIV_DIV2, TIMER_CKDIV_DIV4
                  repetitioncounter: counter repetition value, 0~255
    \param[out] none
    \retval     none
*/
void timer_init(uint32_t timer_periph, timer_parameter_struct *initpara)
{
    uint32_t timer_ctl = TIMER_CTL0(timer_periph);
    uint32_t timer_car = TIMER_CAR(timer_periph);
    uint32_t timer_swevg = TIMER_SWEVG(timer_periph);

    /* configure the counter prescaler value */
    TIMER_PSC(timer_periph) = initpara->prescaler;

    /* configure the counter direction and aligned mode */
    if ((TIMER0 == timer_periph) || (TIMER1 == timer_periph) || (TIMER2 == timer_periph)
        || (TIMER3 == timer_periph) || (TIMER4 == timer_periph) || (TIMER7 == timer_periph)) {
        timer_ctl &= ~(uint32_t)(TIMER_CTL0_DIR | TIMER_CTL0_CAM);
        timer_ctl |= (uint32_t)initpara->alignedmode;
        timer_ctl |= (uint32_t)initpara->counterdirection;
    }

    /* configure the autoreload value */
    timer_car = initpara->period;

    if ((TIMER5 != timer_periph) && (TIMER6 != timer_periph)) {
        /* reset the CKDIV bit */
        timer_ctl &= ~(uint32_t)TIMER_CTL0_CKDIV;
        timer_ctl |= (uint32_t)initpara->clockdivision;
    }

    if ((TIMER0 == timer_periph) || (TIMER7 == timer_periph)) {
        /* configure the repetition counter value */
        TIMER_CREP(timer_periph) = initpara->repetitioncounter;
    }

    /* generate an update event */
    timer_swevg |= (uint32_t)TIMER_SWEVG_UPG;

    TIMER_CTL0(timer_periph) = timer_ctl;
    TIMER_CAR(timer_periph) = timer_car;
    TIMER_SWEVG(timer_periph) = timer_swevg;
}


/*!
    \brief      enable a TIMER
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     none
*/
void timer_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_CEN;
}

/*!
    \brief      disable a TIMER
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     none
*/
void timer_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_CEN;
}

/*!
    \brief      enable the auto reload shadow function
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     none
*/
void timer_auto_reload_shadow_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_ARSE;
}

/*!
    \brief      disable the auto reload shadow function
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     none
*/
void timer_auto_reload_shadow_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_ARSE;
}

/*!
    \brief      enable the update event
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     none
*/
void timer_update_event_enable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_UPDIS;
}

/*!
    \brief      disable the update event
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     none
*/
void timer_update_event_disable(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t) TIMER_CTL0_UPDIS;
}

/*!
    \brief      set TIMER counter alignment mode
    \param[in]  timer_periph: TIMERx(x=0..4,7)
    \param[in]  aligned:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_COUNTER_EDGE: edge-aligned mode
      \arg        TIMER_COUNTER_CENTER_DOWN: center-aligned and counting down assert mode
      \arg        TIMER_COUNTER_CENTER_UP: center-aligned and counting up assert mode
      \arg        TIMER_COUNTER_CENTER_BOTH: center-aligned and counting up/down assert mode
    \param[out] none
    \retval     none
*/
void timer_counter_alignment(uint32_t timer_periph, uint16_t aligned)
{
    TIMER_CTL0(timer_periph) = (TIMER_CTL0(timer_periph) & ~TIMER_CTL0_CAM) | aligned;
}

/*!
    \brief      set TIMER counter up direction
    \param[in]  timer_periph: TIMERx(x=0..4,7)
    \param[out] none
    \retval     none
*/
void timer_counter_up_direction(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) &= ~(uint32_t)TIMER_CTL0_DIR;
}

/*!
    \brief      set TIMER counter down direction
    \param[in]  timer_periph: TIMERx(x=0..4,7)
    \param[out] none
    \retval     none
*/
void timer_counter_down_direction(uint32_t timer_periph)
{
    TIMER_CTL0(timer_periph) |= (uint32_t)TIMER_CTL0_DIR;
}

/*!
    \brief      configure TIMER prescaler
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[in]  prescaler: prescaler value,0~65535
    \param[in]  pscreload: prescaler reload mode
                only one parameter can be selected which is shown as below:
      \arg        TIMER_PSC_RELOAD_NOW: the prescaler is loaded right now
      \arg        TIMER_PSC_RELOAD_UPDATE: the prescaler is loaded at the next update event
    \param[out] none
    \retval     none
*/
void timer_prescaler_config(uint32_t timer_periph, uint16_t prescaler, uint8_t pscreload)
{
    TIMER_PSC(timer_periph) = prescaler;
    if(pscreload == TIMER_PSC_RELOAD_NOW){
        TIMER_SWEVG(timer_periph) |= TIMER_SWEVG_UPG;
    }
}

/*!
    \brief      configure TIMER repetition register value
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[in]  repetition: the counter repetition value,0~255
    \param[out] none
    \retval     none
*/
void timer_repetition_value_config(uint32_t timer_periph, uint16_t repetition)
{
    TIMER_CREP(timer_periph) = (uint32_t)repetition;
} 
 
/*!
    \brief      configure TIMER autoreload register value
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[in]  autoreload: the counter auto-reload value,0~65535
    \param[out] none
    \retval     none
*/
void timer_autoreload_value_config(uint32_t timer_periph, uint16_t autoreload)
{
    TIMER_CAR(timer_periph) = (uint32_t)autoreload;
}

/*!
    \brief      configure TIMER counter register value
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[in]  counter: the counter value,0~65535
    \param[out] none
    \retval     none
*/
void timer_counter_value_config(uint32_t timer_periph, uint16_t counter)
{
    TIMER_CNT(timer_periph) = (uint32_t)counter;
}

/*!
    \brief      read TIMER counter value
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     counter value
*/         
uint32_t timer_counter_read(uint32_t timer_periph)
{
    return TIMER_CNT(timer_periph);
}

/*!
    \brief      read TIMER prescaler value
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[out] none
    \retval     prescaler register value
*/
uint16_t timer_prescaler_read(uint32_t timer_periph)
{
    return (uint16_t)(TIMER_PSC(timer_periph));
}

/*!
    \brief      configure TIMER single pulse mode
    \param[in]  timer_periph: TIMERx(x=0..8,11)
    \param[in]  spmode: TIMER_SP_MODE_SINGLE or TIMER_SP_MODE_REPETITIVE
    \param[out] none
    \retval     none
*/
void timer_single_pulse_mode_config(uint32_t timer_periph, uint32_t spmode)
{
    TIMER_CTL0(timer_periph) ^= ((spmode == TIMER_SP_MODE_SINGLE)
        ? (uint32_t)(~TIMER_CTL0_SPM)
        : (uint32_t)TIMER_CTL0_SPM);
}


/*!
    \brief      configure TIMER update source 
    \param[in]  timer_periph: TIMERx(x=0..13)
    \param[in]  update: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_UPDATE_SRC_GLOBAL: update generate by setting of UPG bit or the counter overflow/underflow,or the slave mode controller trigger
      \arg        TIMER_UPDATE_SRC_REGULAR: update generate only by counter overflow/underflow
    \param[out] none
    \retval     none
*/
void timer_update_source_config(uint32_t timer_periph, uint32_t update)
{
    uint32_t ctl0 = TIMER_CTL0(timer_periph);
    ctl0 &= ~TIMER_CTL0_UPS;
    ctl0 |= (update == TIMER_UPDATE_SRC_REGULAR) ? TIMER_CTL0_UPS : 0;
    TIMER_CTL0(timer_periph) = ctl0;
}

/*!
    \brief      enable the TIMER interrupt
    \param[in]  timer_periph: please refer to the following parameters 
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
    \param[out] none
    \retval     none
*/
void timer_interrupt_enable(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_DMAINTEN(timer_periph) |= (uint32_t)interrupt; 
}

/*!
    \brief      disable the TIMER interrupt
    \param[in]  timer_periph: please refer to the following parameters
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
    \param[out] none
    \retval     none
*/
void timer_interrupt_disable(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_DMAINTEN(timer_periph) &= (~(uint32_t)interrupt); 
}

/*!
    \brief      get timer interrupt flag
    \param[in]  timer_periph: please refer to the following parameters
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
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus timer_interrupt_flag_get(uint32_t timer_periph, uint32_t interrupt)
{
    return (((TIMER_INTF(timer_periph) & interrupt) != 0U) && ((TIMER_DMAINTEN(timer_periph) & interrupt) != 0U)) ? SET : RESET;
}

/*!
    \brief      clear TIMER interrupt flag
    \param[in]  timer_periph: please refer to the following parameters
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
    \param[out] none
    \retval     none
*/
void timer_interrupt_flag_clear(uint32_t timer_periph, uint32_t interrupt)
{
    TIMER_INTF(timer_periph) = (~(uint32_t)interrupt);
}

/*!
    \brief      get TIMER flags
    \param[in]  timer_periph: please refer to the following parameters
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
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus timer_flag_get(uint32_t timer_periph, uint32_t flag)
{
    return ((TIMER_INTF(timer_periph) & flag) != 0U) ? SET : RESET;
}

/*!
    \brief      clear TIMER flags
    \param[in]  timer_periph: please refer to the following parameters
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
    \param[out] none
    \retval     none
*/
void timer_flag_clear(uint32_t timer_periph, uint32_t flag)
{
    TIMER_INTF(timer_periph) = (~(uint32_t)flag);
}

/*!
    \brief      enable the TIMER DMA
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  dma: specify which DMA to enable
                only one parameter can be selected which is shown as below:
      \arg        TIMER_DMA_UPD:  update DMA enable,TIMERx(x=0..7)
      \arg        TIMER_DMA_CH0D: channel 0 DMA enable,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CH1D: channel 1 DMA enable,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CH2D: channel 2 DMA enable,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CH3D: channel 3 DMA enable,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CMTD: commutation DMA request enable,TIMERx(x=0,7)
      \arg        TIMER_DMA_TRGD: trigger DMA enable,TIMERx(x=0..4,7)
    \param[out] none
    \retval     none
*/
void timer_dma_enable(uint32_t timer_periph, uint16_t dma)
{
    TIMER_DMAINTEN(timer_periph) |= (uint32_t) dma; 
}

/*!
    \brief      disable the TIMER DMA
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  dma: specify which DMA to enable
                one or more parameters can be selected which are shown as below:
      \arg        TIMER_DMA_UPD:  update DMA ,TIMERx(x=0..7)
      \arg        TIMER_DMA_CH0D: channel 0 DMA request,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CH1D: channel 1 DMA request,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CH2D: channel 2 DMA request,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CH3D: channel 3 DMA request,TIMERx(x=0..4,7)
      \arg        TIMER_DMA_CMTD: commutation DMA request ,TIMERx(x=0,7)
      \arg        TIMER_DMA_TRGD: trigger DMA request,TIMERx(x=0..4,7)
    \param[out] none
    \retval     none
*/
void timer_dma_disable(uint32_t timer_periph, uint16_t dma)
{
    TIMER_DMAINTEN(timer_periph) &= (~(uint32_t)(dma)); 
}

/*!
    \brief      channel DMA request source selection
    \param[in]  timer_periph: TIMERx(x=0..4,7)
    \param[in]  dma_request: channel DMA request source selection
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMAREQUEST_CHANNELEVENT: DMA request of channel y is sent when channel y event occurs
       \arg        TIMER_DMAREQUEST_UPDATEEVENT: DMA request of channel y is sent when update event occurs 
    \param[out] none
    \retval     none
*/
void timer_channel_dma_request_source_select(uint32_t timer_periph, uint8_t dma_request)
{
    TIMER_CTL1(timer_periph) ^= (-(dma_request ^ TIMER_DMAREQUEST_UPDATEEVENT) & TIMER_CTL1_DMAS);
}

/*!
    \brief      configure the TIMER DMA transfer
    \param[in]  timer_periph: please refer to the following parameters
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
                only one parameter can be selected
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMACFG_DMATC_xTRANSFER(x=1..18): DMA transfer x time
    \param[out] none
    \retval     none
*/
void timer_dma_transfer_config(uint32_t timer_periph, uint32_t dma_baseaddr, uint32_t dma_lenth)
{
    TIMER_DMACFG(timer_periph) = (TIMER_DMACFG(timer_periph) & ~(TIMER_DMACFG_DMATA | TIMER_DMACFG_DMATC))
                                 | (dma_baseaddr | dma_lenth);
}

/*!
    \brief      software generate events 
    \param[in]  timer_periph: please refer to the following parameters
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
    \param[out] none
    \retval     none
*/
void timer_event_software_generate(uint32_t timer_periph, uint16_t event)
{
    TIMER_SWEVG(timer_periph) |= (uint32_t)event;
}

/*!
    \brief      initialize TIMER break parameter struct with a default value
    \param[out] breakpara: TIMER break parameter struct
    \retval     none
*/
void timer_break_struct_para_init(timer_break_parameter_struct* breakpara)
{
    /* initialize the break parameter struct member with the default value */
    breakpara->runoffstate     = TIMER_ROS_STATE_DISABLE;
    breakpara->ideloffstate    = TIMER_IOS_STATE_DISABLE;
    breakpara->deadtime        = 0U;
    breakpara->breakpolarity   = TIMER_BREAK_POLARITY_LOW;
    breakpara->outputautostate = TIMER_OUTAUTO_DISABLE;
    breakpara->protectmode     = TIMER_CCHP_PROT_OFF;
    breakpara->breakstate      = TIMER_BREAK_DISABLE;
}

/*!
    \brief      configure TIMER break function 
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[in]  breakpara: TIMER break parameter struct
                runoffstate: TIMER_ROS_STATE_ENABLE,TIMER_ROS_STATE_DISABLE
                ideloffstate: TIMER_IOS_STATE_ENABLE,TIMER_IOS_STATE_DISABLE
                deadtime: 0~255
                breakpolarity: TIMER_BREAK_POLARITY_LOW,TIMER_BREAK_POLARITY_HIGH
                outputautostate: TIMER_OUTAUTO_ENABLE,TIMER_OUTAUTO_DISABLE
                protectmode: TIMER_CCHP_PROT_OFF,TIMER_CCHP_PROT_0,TIMER_CCHP_PROT_1,TIMER_CCHP_PROT_2
                breakstate: TIMER_BREAK_ENABLE,TIMER_BREAK_DISABLE
    \param[out] none
    \retval     none
*/
void timer_break_config(uint32_t timer_periph, timer_break_parameter_struct* breakpara)
{
    TIMER_CCHP(timer_periph) = (uint32_t)(((uint32_t)(breakpara->runoffstate))|
                                          ((uint32_t)(breakpara->ideloffstate))|
                                          ((uint32_t)(breakpara->deadtime))|
                                          ((uint32_t)(breakpara->breakpolarity))|
                                          ((uint32_t)(breakpara->outputautostate)) |
                                          ((uint32_t)(breakpara->protectmode))|
                                          ((uint32_t)(breakpara->breakstate))) ;
}

/*!
    \brief      enable TIMER break function
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[out] none
    \retval     none
*/
void timer_break_enable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_BRKEN;
}

/*!
    \brief      disable TIMER break function
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[out] none
    \retval     none
*/
void timer_break_disable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) &= ~(uint32_t)TIMER_CCHP_BRKEN;
}

/*!
    \brief      enable TIMER output automatic function
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[out] none
    \retval     none
*/
void timer_automatic_output_enable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_OAEN;
}

/*!
    \brief      disable TIMER output automatic function
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[out] none
    \retval     none
*/
void timer_automatic_output_disable(uint32_t timer_periph)
{
    TIMER_CCHP(timer_periph) &= ~(uint32_t)TIMER_CCHP_OAEN;
}

/*!
    \brief      configure TIMER primary output function
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[in]  newvalue: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void timer_primary_output_config(uint32_t timer_periph, ControlStatus newvalue)
{
    if(ENABLE == newvalue){
        TIMER_CCHP(timer_periph) |= (uint32_t)TIMER_CCHP_POEN;
    }else{
        TIMER_CCHP(timer_periph) &= (~(uint32_t)TIMER_CCHP_POEN);
    }
}

/*!
    \brief      enable or disable channel capture/compare control shadow register
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[in]  newvalue: ENABLE or DISABLE 
    \param[out] none
    \retval     none
*/
void timer_channel_control_shadow_config(uint32_t timer_periph, ControlStatus newvalue)
{
    TIMER_CTL1(timer_periph) ^= (-newvalue ^ TIMER_CTL1(timer_periph)) & TIMER_CTL1_CCSE;
}

/*!
    \brief      configure TIMER channel control shadow register update control
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[in]  ccuctl: channel control shadow register update control
                only one parameter can be selected which is shown as below:
      \arg        TIMER_UPDATECTL_CCU: the shadow registers update by when CMTG bit is set
      \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers update by when CMTG bit is set or an rising edge of TRGI occurs 
    \param[out] none
    \retval     none
*/              
void timer_channel_control_shadow_update_config(uint32_t timer_periph, uint8_t ccuctl)
{
    uint32_t ctl1 = TIMER_CTL1(timer_periph);
    if(TIMER_UPDATECTL_CCU == ccuctl){
        TIMER_CTL1(timer_periph) = ctl1 & ~TIMER_CTL1_CCUC;
    }else if(TIMER_UPDATECTL_CCUTRI == ccuctl){
        TIMER_CTL1(timer_periph) = ctl1 | TIMER_CTL1_CCUC;
    }else{
        /* illegal parameters */
    }
}

/*!
    \brief      initialize TIMER channel output parameter struct with a default value
    \param[in]  ocpara: TIMER channel n output parameter struct
    \param[out] none
    \retval     none
*/
void timer_channel_output_struct_para_init(timer_oc_parameter_struct* ocpara)
{
    /* initialize the channel output parameter struct member with the default value */
    ocpara->outputstate  = (uint16_t)TIMER_CCX_DISABLE;
    ocpara->outputnstate = TIMER_CCXN_DISABLE;
    ocpara->ocpolarity   = TIMER_OC_POLARITY_HIGH;
    ocpara->ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    ocpara->ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    ocpara->ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
}

/*!
    \brief      configure TIMER channel output function
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel 0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel 1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel 2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel 3(TIMERx(x=0..4,7))
    \param[in]  ocpara: TIMER channeln output parameter struct
                outputstate: TIMER_CCX_ENABLE,TIMER_CCX_DISABLE
                outputnstate: TIMER_CCXN_ENABLE,TIMER_CCXN_DISABLE
                ocpolarity: TIMER_OC_POLARITY_HIGH,TIMER_OC_POLARITY_LOW
                ocnpolarity: TIMER_OCN_POLARITY_HIGH,TIMER_OCN_POLARITY_LOW
                ocidlestate: TIMER_OC_IDLE_STATE_LOW,TIMER_OC_IDLE_STATE_HIGH
                ocnidlestate: TIMER_OCN_IDLE_STATE_LOW,TIMER_OCN_IDLE_STATE_HIGH
    \param[out] none
    \retval     none
*/
void timer_channel_output_config(uint32_t timer_periph, uint16_t channel, timer_oc_parameter_struct* ocpara)
{
    switch(channel){
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        /* reset the CH0EN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
        TIMER_CHCTL0(timer_periph) &= ~(uint32_t)TIMER_CHCTL0_CH0MS;
        /* set the CH0EN bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)ocpara->outputstate;
        /* reset the CH0P bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0P);
        /* set the CH0P bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)ocpara->ocpolarity;

        if((TIMER0 == timer_periph) || (TIMER7 == timer_periph)){
            /* reset the CH0NEN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NEN);
            /* set the CH0NEN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)ocpara->outputnstate;
            /* reset the CH0NP bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH0NP);
            /* set the CH0NP bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)ocpara->ocnpolarity;
            /* reset the ISO0 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO0);
            /* set the ISO0 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)ocpara->ocidlestate;
            /* reset the ISO0N bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO0N);
            /* set the ISO0N bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)ocpara->ocnidlestate;
        }
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        /* reset the CH1EN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
        TIMER_CHCTL0(timer_periph) &= ~(uint32_t)TIMER_CHCTL0_CH1MS;
        /* set the CH1EN bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocpara->outputstate << 4U);
        /* reset the CH1P bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1P);
        /* set the CH1P bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocpolarity) << 4U);

        if((TIMER0 == timer_periph) || (TIMER7 == timer_periph)){
            /* reset the CH1NEN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NEN);
            /* set the CH1NEN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->outputnstate) << 4U);
            /* reset the CH1NP bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH1NP);
            /* set the CH1NP bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnpolarity) << 4U);
            /* reset the ISO1 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO1);
            /* set the ISO1 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocidlestate) << 2U);
            /* reset the ISO1N bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO1N);
            /* set the ISO1N bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnidlestate) << 2U);
        }
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        /* reset the CH2EN bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2EN);
        TIMER_CHCTL1(timer_periph) &= ~(uint32_t)TIMER_CHCTL1_CH2MS;
        /* set the CH2EN bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocpara->outputstate << 8U);
        /* reset the CH2P bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2P);
        /* set the CH2P bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocpolarity) << 8U);

        if((TIMER0 == timer_periph) || (TIMER7 == timer_periph)){
            /* reset the CH2NEN bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NEN);
            /* set the CH2NEN bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->outputnstate) << 8U);
            /* reset the CH2NP bit */
            TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH2NP);
            /* set the CH2NP bit */
            TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnpolarity) << 8U);
            /* reset the ISO2 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO2);
            /* set the ISO2 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocidlestate) << 4U);
            /* reset the ISO2N bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO2N);
            /* set the ISO2N bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocnidlestate) << 4U);
        }
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        /* reset the CH3EN bit */
        TIMER_CHCTL2(timer_periph) &=(~(uint32_t)TIMER_CHCTL2_CH3EN);
        TIMER_CHCTL1(timer_periph) &= ~(uint32_t)TIMER_CHCTL1_CH3MS;
        /* set the CH3EN bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)ocpara->outputstate << 12U);
        /* reset the CH3P bit */
        TIMER_CHCTL2(timer_periph) &= (~(uint32_t)TIMER_CHCTL2_CH3P);
        /* set the CH3P bit */
        TIMER_CHCTL2(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocpolarity) << 12U);

        if((TIMER0 == timer_periph) || (TIMER7 == timer_periph)){
            /* reset the ISO3 bit */
            TIMER_CTL1(timer_periph) &= (~(uint32_t)TIMER_CTL1_ISO3);
            /* set the ISO3 bit */
            TIMER_CTL1(timer_periph) |= (uint32_t)((uint32_t)(ocpara->ocidlestate) << 6U);
        }
        break;
    default:
        break;
    }
}

/*!
    \brief      configure TIMER channel output compare mode
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \param[in]  ocmode: channel output compare mode
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_MODE_TIMING: timing mode
      \arg        TIMER_OC_MODE_ACTIVE: active mode
      \arg        TIMER_OC_MODE_INACTIVE: inactive mode
      \arg        TIMER_OC_MODE_TOGGLE: toggle mode
      \arg        TIMER_OC_MODE_LOW: force low mode
      \arg        TIMER_OC_MODE_HIGH: force high mode
      \arg        TIMER_OC_MODE_PWM0: PWM0 mode
      \arg        TIMER_OC_MODE_PWM1: PWM1 mode
    \param[out] none
    \retval     none
*/
void timer_channel_output_mode_config(uint32_t timer_periph, uint16_t channel, uint16_t ocmode)
{
    uint32_t chctl0_register = TIMER_CHCTL0(timer_periph);
    uint32_t chctl1_register = TIMER_CHCTL1(timer_periph);
    uint32_t chctl0_mask = TIMER_CHCTL0_CH0COMCTL | TIMER_CHCTL0_CH1COMCTL;
    uint32_t chctl1_mask = TIMER_CHCTL1_CH2COMCTL | TIMER_CHCTL1_CH3COMCTL;
    uint32_t ocmode_shift = (channel << 3U);

    chctl0_register &= ~chctl0_mask;
    chctl1_register &= ~chctl1_mask;

    if(channel < 2U){
        chctl0_register |= (uint32_t)(ocmode << ocmode_shift);
    }else{
        chctl1_register |= (uint32_t)(ocmode << ocmode_shift);
    }

    TIMER_CHCTL0(timer_periph) = chctl0_register;
    TIMER_CHCTL1(timer_periph) = chctl1_register;
}

/*!
    \brief      configure TIMER channel output pulse value
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \param[in]  pulse: channel output pulse value,0~65535
    \param[out] none
    \retval     none
*/
void timer_channel_output_pulse_value_config(uint32_t timer_periph, uint16_t channel, uint32_t pulse)
{
    uint32_t chcv_register;

    /* get channel capture compare register address */
    chcv_register = TIMER_CH0CV(timer_periph) + channel;

    /* configure TIMER channel capture compare register value */
    *(uint32_t *)chcv_register = pulse;
}

/*!
    \brief      configure TIMER channel output shadow function
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \param[in]  ocshadow: channel output shadow state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_SHADOW_ENABLE: channel output shadow state enable
      \arg        TIMER_OC_SHADOW_DISABLE: channel output shadow state disable
    \param[out] none
    \retval     none
*/
void timer_channel_output_shadow_config(uint32_t timer_periph, uint16_t channel, uint16_t ocshadow)
{
    uint32_t chctl_reg = (channel < 2U) ? TIMER_CHCTL0(timer_periph) : TIMER_CHCTL1(timer_periph);
    uint32_t mask = (channel < 2U) ? TIMER_CHCTL0_CH0COMSEN << (channel * 8U) : TIMER_CHCTL1_CH2COMSEN << (channel * 8U);
    
    chctl_reg &= ~mask;
    chctl_reg |= (uint32_t)ocshadow << (channel * 8U);
    
    if(channel < 2U){
        TIMER_CHCTL0(timer_periph) = chctl_reg;
    }else{
        TIMER_CHCTL1(timer_periph) = chctl_reg;
    }
}

/*!
    \brief      configure TIMER channel output fast function
    \param[in]  timer_periph: please refer to the following parameters
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
    \param[out] none
    \retval     none
*/
void timer_channel_output_fast_config(uint32_t timer_periph, uint16_t channel, uint16_t ocfast)
{
    uint32_t chctl_reg;
    uint32_t mask;

    chctl_reg = (channel < 2U) ? TIMER_CHCTL0(timer_periph) : TIMER_CHCTL1(timer_periph);
    mask = (channel < 2U) ? TIMER_CHCTL0_CH0COMFEN << (channel * 8U) : TIMER_CHCTL1_CH2COMFEN << (channel * 8U);

    chctl_reg &= ~mask;
    chctl_reg |= (uint32_t)ocfast << (channel * 8U);

    if(channel < 2U){
        TIMER_CHCTL0(timer_periph) = chctl_reg;
    }else{
        TIMER_CHCTL1(timer_periph) = chctl_reg;
    }
}

/*!
    \brief      configure TIMER channel output clear function
    \param[in]  timer_periph: TIMERx(x=0..4,7)
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0
      \arg        TIMER_CH_1: TIMER channel1
      \arg        TIMER_CH_2: TIMER channel2
      \arg        TIMER_CH_3: TIMER channel3
    \param[in]  occlear: channel output clear function
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_CLEAR_ENABLE: channel output clear function enable
      \arg        TIMER_OC_CLEAR_DISABLE: channel output clear function disable
    \param[out] none
    \retval     none
*/
void timer_channel_output_clear_config(uint32_t timer_periph, uint16_t channel, uint16_t occlear)
{
    uint32_t mask;
    uint32_t chctl_reg;

    mask = (channel < 2U) ? TIMER_CHCTL0_CH0COMCEN << (channel * 8U) : TIMER_CHCTL1_CH2COMCEN << (channel * 8U);
    chctl_reg = (channel < 2U) ? TIMER_CHCTL0(timer_periph) : TIMER_CHCTL1(timer_periph);

    chctl_reg &= ~mask;
    chctl_reg |= (uint32_t)occlear << (channel * 8U);

    if(channel < 2U){
        TIMER_CHCTL0(timer_periph) = chctl_reg;
    }else{
        TIMER_CHCTL1(timer_periph) = chctl_reg;
    }
}

/*!
    \brief      configure TIMER channel output polarity 
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \param[in]  ocpolarity: channel output polarity 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OC_POLARITY_HIGH: channel output polarity is high
      \arg        TIMER_OC_POLARITY_LOW: channel output polarity is low
    \param[out] none
    \retval     none
*/
void timer_channel_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocpolarity)
{
    uint32_t bit_pos = (channel & 0x3) << 2U;
    uint32_t chctl_reg = TIMER_CHCTL2(timer_periph);

    chctl_reg &= ~(TIMER_CHCTL2_CH0P << bit_pos);
    chctl_reg |= (uint32_t)ocpolarity << bit_pos;

    TIMER_CHCTL2(timer_periph) = chctl_reg;
}

/*!
    \brief      configure TIMER channel complementary output polarity 
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,7))
    \param[in]  ocnpolarity: channel complementary output polarity 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OCN_POLARITY_HIGH: channel complementary output polarity is high
      \arg        TIMER_OCN_POLARITY_LOW: channel complementary output polarity is low
    \param[out] none
    \retval     none
*/
void timer_channel_complementary_output_polarity_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnpolarity)
{
    uint32_t offset = (channel & 0x3) << 2U;
    uint32_t mask = TIMER_CHCTL2_CH0NP << offset;

    TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & ~mask) | (uint32_t)ocnpolarity << offset;
}

/*!
    \brief      configure TIMER channel enable state
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \param[in]  state: TIMER channel enable state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CCX_ENABLE: channel enable 
      \arg        TIMER_CCX_DISABLE: channel disable 
    \param[out] none
    \retval     none
*/
void timer_channel_output_state_config(uint32_t timer_periph, uint16_t channel, uint32_t state)
{
    uint32_t pos = (channel & 0x3) << 2U;
    uint32_t mask = TIMER_CHCTL2_CH0EN << pos;

    TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & ~mask) | (state << pos);
}

/*!
    \brief      configure TIMER channel complementary output enable state
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,7))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,7))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,7))
    \param[in]  ocnstate: TIMER channel complementary output enable state
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CCXN_ENABLE: channel complementary enable 
      \arg        TIMER_CCXN_DISABLE: channel complementary disable 
    \param[out] none
    \retval     none
*/
void timer_channel_complementary_output_state_config(uint32_t timer_periph, uint16_t channel, uint16_t ocnstate)
{
    uint32_t shift = channel * 4U;
    uint32_t mask = ~(TIMER_CHCTL2_CH0NEN << shift);
    uint32_t value = ocnstate << shift;

    TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & mask) | value;
}

/*!
    \brief      initialize TIMER channel input parameter struct with a default value
    \param[in]  icpara: TIMER channel intput parameter struct
    \param[out] none
    \retval     none
*/
void timer_channel_input_struct_para_init(timer_ic_parameter_struct* icpara)
{
    /* initialize the channel input parameter struct member with the default value */
    icpara->icpolarity  = TIMER_IC_POLARITY_RISING;
    icpara->icselection = TIMER_IC_SELECTION_DIRECTTI;
    icpara->icprescaler = TIMER_IC_PSC_DIV1;
    icpara->icfilter    = 0U;
}

/*!
    \brief      configure TIMER input capture parameter 
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
     \param[in]  icpara: TIMER channel intput parameter struct
                 icpolarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING
                 icselection: TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI,TIMER_IC_SELECTION_ITS
                 icprescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
                 icfilter: 0~15
    \param[out]  none
    \retval      none
*/
void timer_input_capture_config(uint32_t timer_periph, uint16_t channel, timer_ic_parameter_struct *icpara)
{
    uint32_t icpolarity_shift = channel * 4U;
    uint32_t icselection_shift = channel * 8U;
    uint32_t icfilter_shift = channel * 4U + 4U;

    uint32_t icpolarity = (uint32_t)(icpara->icpolarity) << icpolarity_shift;
    uint32_t icselection = (uint32_t)(icpara->icselection) << icselection_shift;
    uint32_t icfilter = (uint32_t)(icpara->icfilter) << icfilter_shift;

    switch(channel){
        case TIMER_CH_0:
            TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & ~(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP)) | icpolarity;
            TIMER_CHCTL0(timer_periph) = (TIMER_CHCTL0(timer_periph) & ~(TIMER_CHCTL0_CH0MS | TIMER_CHCTL0_CH0CAPFLT)) | icselection | icfilter;
            TIMER_CHCTL2(timer_periph) |= TIMER_CHCTL2_CH0EN;
            break;

        case TIMER_CH_1:
            TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & ~(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP)) | (icpolarity << 4U);
            TIMER_CHCTL0(timer_periph) = (TIMER_CHCTL0(timer_periph) & ~(TIMER_CHCTL0_CH1MS | TIMER_CHCTL0_CH1CAPFLT)) | (icselection << 8U) | (icfilter << 12U);
            TIMER_CHCTL2(timer_periph) |= TIMER_CHCTL2_CH1EN;
            break;

        case TIMER_CH_2:
            TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & ~(TIMER_CHCTL2_CH2EN | TIMER_CHCTL2_CH2P | TIMER_CHCTL2_CH2NP)) | (icpolarity << 8U);
            TIMER_CHCTL1(timer_periph) = (TIMER_CHCTL1(timer_periph) & ~(TIMER_CHCTL1_CH2MS | TIMER_CHCTL1_CH2CAPFLT)) | (icselection << 16U) | (icfilter << 20U);
            TIMER_CHCTL2(timer_periph) |= TIMER_CHCTL2_CH2EN;
            break;

        case TIMER_CH_3:
            TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & ~(TIMER_CHCTL2_CH3EN | TIMER_CHCTL2_CH3P)) | (icpolarity << 12U);
            TIMER_CHCTL1(timer_periph) = (TIMER_CHCTL1(timer_periph) & ~(TIMER_CHCTL1_CH3MS | TIMER_CHCTL1_CH3CAPFLT)) | (icselection << 24U) | (icfilter << 28U);
            TIMER_CHCTL2(timer_periph) |= TIMER_CHCTL2_CH3EN;
            break;

        default:
            break;
    }

    /* configure TIMER channel input capture prescaler value */
    timer_channel_input_capture_prescaler_config(timer_periph, channel, (uint16_t)(icpara->icprescaler));
}

/*!
    \brief      configure TIMER channel input capture prescaler value
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \param[in]  prescaler: channel input capture prescaler value
                only one parameter can be selected which is shown as below:
      \arg        TIMER_IC_PSC_DIV1: no prescaler
      \arg        TIMER_IC_PSC_DIV2: divided by 2
      \arg        TIMER_IC_PSC_DIV4: divided by 4
      \arg        TIMER_IC_PSC_DIV8: divided by 8
    \param[out] none
    \retval     none
*/
void timer_channel_input_capture_prescaler_config(uint32_t timer_periph, uint16_t channel, uint16_t prescaler)
{
    uint32_t shift = (channel & 0x3) << 3U;
    uint32_t mask = (TIMER_CHCTL0_CH0CAPPSC | TIMER_CHCTL0_CH1CAPPSC | TIMER_CHCTL1_CH2CAPPSC | TIMER_CHCTL1_CH3CAPPSC) << shift;

    TIMER_CHCTL0(timer_periph) = (TIMER_CHCTL0(timer_periph) & ~mask) | ((uint32_t)prescaler << shift);
}

/*!
    \brief      read TIMER channel capture compare register value
    \param[in]  timer_periph: please refer to the following parameters
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0..4,7..13))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0..4,7,8,11))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0..4,7))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0..4,7))
    \param[out] none
    \retval     channel capture compare register value
*/
uint32_t timer_channel_capture_value_register_read(uint32_t timer_periph, uint16_t channel)
{
    uint32_t chcv_register;

    chcv_register = (channel & TIMER_CH_0) ? TIMER_CH0CV(timer_periph) :
                    (channel & TIMER_CH_1) ? TIMER_CH1CV(timer_periph) :
                    (channel & TIMER_CH_2) ? TIMER_CH2CV(timer_periph) :
                    TIMER_CH3CV(timer_periph);

    return (*(uint32_t *)chcv_register);
}

/*!
    \brief      configure TIMER input pwm capture function 
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0
      \arg        TIMER_CH_1: TIMER channel1
     \param[in]  icpwm:TIMER channel intput pwm parameter struct
                 icpolarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING
                 icselection: TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI
                 icprescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
                 icfilter: 0~15
    \param[out] none
    \retval     none
*/
void timer_input_pwm_capture_config(uint32_t timer_periph, uint16_t channel, timer_ic_parameter_struct* icpwm)
{
    uint32_t chctl2 = TIMER_CHCTL2(timer_periph);
    uint32_t chctl0 = TIMER_CHCTL0(timer_periph);
    uint32_t ch0en = TIMER_CHCTL2_CH0EN;
    uint32_t ch1en = TIMER_CHCTL2_CH1EN;
    uint32_t ch0capflt = ((uint32_t)(icpwm->icfilter) << 4U);
    uint32_t ch1capflt = ((uint32_t)(icpwm->icfilter) << 12U);
    uint32_t ch0p = (uint32_t)(icpwm->icpolarity);
    uint32_t ch1p = (uint32_t)((uint32_t)(icpwm->icpolarity) << 4U);
    uint32_t ch0ms = (uint32_t)(icpwm->icselection);
    uint32_t ch1ms = (uint32_t)((uint32_t)(icpwm->icselection) << 8U);
    uint32_t ch0prescaler = (uint16_t)(icpwm->icprescaler);
    uint32_t ch1prescaler = (uint32_t)(ch0prescaler << 16U);

    /* Reset channel control 2 register */
    TIMER_CHCTL2(timer_periph) = 0U;
    /* Reset channel control 0 register */
    TIMER_CHCTL0(timer_periph) = 0U;

    /* Set channel input polarity */
    if (TIMER_IC_POLARITY_RISING == icpwm->icpolarity) {
        ch0p = TIMER_IC_POLARITY_FALLING;
    } else {
        ch1p = TIMER_IC_POLARITY_RISING;
    }

    /* Set channel input pwm parameter */
    if (TIMER_IC_SELECTION_DIRECTTI == icpwm->icselection) {
        ch0ms = TIMER_IC_SELECTION_INDIRECTTI;
        ch1ms = TIMER_IC_SELECTION_INDIRECTTI;
    }

    if (TIMER_CH_0 == channel) {
        chctl2 |= ch1en | ch1p | ch1ms | ch1capflt | ch1prescaler;
        chctl0 |= ch0en | ch0p | ch0ms | ch0capflt | ch0prescaler;
    } else {
        chctl2 |= ch0en | ch0p | ch0ms | ch0capflt | ch0prescaler;
        chctl0 |= ch1en | ch1p | ch1ms | ch1capflt | ch1prescaler;
    }

    TIMER_CHCTL2(timer_periph) = chctl2;
    TIMER_CHCTL0(timer_periph) = chctl0;
}

/*!
    \brief      configure TIMER hall sensor mode
    \param[in]  timer_periph: TIMERx(x=0..4,7)
    \param[in]  hallmode: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_HALLINTERFACE_ENABLE: TIMER hall sensor mode enable
      \arg        TIMER_HALLINTERFACE_DISABLE: TIMER hall sensor mode disable
    \param[out] none
    \retval     none
*/
void timer_hall_mode_config(uint32_t timer_periph, uint32_t hallmode)
{
    uint32_t ctl1 = TIMER_CTL1(timer_periph);
    if(TIMER_HALLINTERFACE_ENABLE == hallmode){
        TIMER_CTL1(timer_periph) = ctl1 | TIMER_CTL1_TI0S;
    }else if(TIMER_HALLINTERFACE_DISABLE == hallmode){
        TIMER_CTL1(timer_periph) = ctl1 & ~TIMER_CTL1_TI0S;
    }
}

/*!
    \brief      select TIMER input trigger source 
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
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
    \param[out] none
    \retval     none
*/
void timer_input_trigger_source_select(uint32_t timer_periph, uint32_t intrigger)
{
    TIMER_SMCFG(timer_periph) = (TIMER_SMCFG(timer_periph) & ~TIMER_SMCFG_TRGS) | intrigger;
}

/*!
    \brief      select TIMER master mode output trigger source 
    \param[in]  timer_periph: TIMERx(x=0..7)
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
    \param[out] none
    \retval     none
*/
void timer_master_output_trigger_source_select(uint32_t timer_periph, uint32_t outrigger)
{
    TIMER_CTL1(timer_periph) = (TIMER_CTL1(timer_periph) & ~TIMER_CTL1_MMC) | outrigger;
}

/*!
    \brief      select TIMER slave mode 
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
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
    \param[out] none
    \retval     none
*/
void timer_slave_mode_select(uint32_t timer_periph, uint32_t slavemode)
{
    TIMER_SMCFG(timer_periph) = (TIMER_SMCFG(timer_periph) & ~TIMER_SMCFG_SMC) | (slavemode & TIMER_SMCFG_SMC);
}

/*!
    \brief      configure TIMER master slave mode 
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
    \param[in]  masterslave:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_MASTER_SLAVE_MODE_ENABLE: master slave mode enable
      \arg        TIMER_MASTER_SLAVE_MODE_DISABLE: master slave mode disable
    \param[out] none
    \retval     none
*/ 
void timer_master_slave_mode_config(uint32_t timer_periph, uint32_t masterslave)
{
    uint32_t reg;
    reg = TIMER_SMCFG(timer_periph);
    reg &= ~TIMER_SMCFG_MSM;
    reg |= (masterslave & TIMER_SMCFG_MSM);
    TIMER_SMCFG(timer_periph) = reg;
}

/*!
    \brief      configure TIMER external trigger input
    \param[in]  timer_periph: TIMERx(x=0..4,7)
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
    \param[out] none
    \retval     none
*/
void timer_external_trigger_config(uint32_t timer_periph, uint32_t extprescaler,
                                   uint32_t extpolarity, uint32_t extfilter)
{
    uint32_t reg = TIMER_SMCFG(timer_periph);
    reg &= ~(TIMER_SMCFG_ETP | TIMER_SMCFG_ETPSC | TIMER_SMCFG_ETFC);
    reg |= (extprescaler | extpolarity | (extfilter << 8U));
    TIMER_SMCFG(timer_periph) = reg;
}

/*!
    \brief      configure TIMER quadrature decoder mode
    \param[in]  timer_periph: TIMERx(x=0..4,7)
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
    \param[out] none
    \retval     none
*/
void timer_quadrature_decoder_mode_config(uint32_t timer_periph, uint32_t decomode,
                                   uint16_t ic0polarity, uint16_t ic1polarity)
{
    TIMER_SMCFG(timer_periph) = (TIMER_SMCFG(timer_periph) & ~TIMER_SMCFG_SMC) | decomode;
    TIMER_CHCTL0(timer_periph) = (TIMER_CHCTL0(timer_periph) & ~(TIMER_CHCTL0_CH0MS | TIMER_CHCTL0_CH1MS)) | TIMER_IC_SELECTION_DIRECTTI | (TIMER_IC_SELECTION_DIRECTTI << 8U);
    TIMER_CHCTL2(timer_periph) = (TIMER_CHCTL2(timer_periph) & ~(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP | TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP)) | (ic0polarity | (ic1polarity << 4U));
}

/*!
    \brief      configure TIMER internal clock mode
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
    \param[out] none
    \retval     none
*/
void timer_internal_clock_config(uint32_t timer_periph)
{
    TIMER_SMCFG(timer_periph) &= ~(uint32_t)TIMER_SMCFG_SMC;
}

/*!
    \brief      configure TIMER the internal trigger as external clock input
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
    \param[in]  intrigger:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SMCFG_TRGSEL_ITI0: internal trigger 0
      \arg        TIMER_SMCFG_TRGSEL_ITI1: internal trigger 1
      \arg        TIMER_SMCFG_TRGSEL_ITI2: internal trigger 2
      \arg        TIMER_SMCFG_TRGSEL_ITI3: internal trigger 3
    \param[out] none
    \retval     none
*/
void timer_internal_trigger_as_external_clock_config(uint32_t timer_periph, uint32_t intrigger)
{
    TIMER_SMCFG(timer_periph) = (TIMER_SMCFG(timer_periph) & ~(TIMER_SMCFG_TRGS | TIMER_SMCFG_SMC)) | (intrigger | TIMER_SLAVE_MODE_EXTERNAL0);
}

/*!
    \brief      configure TIMER the external trigger as external clock input
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
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
    \param[out] none
    \retval     none
*/
void timer_external_trigger_as_external_clock_config(uint32_t timer_periph, uint32_t extrigger,
                                       uint16_t extpolarity, uint32_t extfilter)
{
    uint32_t chctl0 = TIMER_CHCTL0(timer_periph);
    uint32_t chctl2 = TIMER_CHCTL2(timer_periph);

    if(TIMER_SMCFG_TRGSEL_CI1FE1 == extrigger){
        chctl2 &= ~(TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP);
        chctl2 |= (uint32_t)extpolarity << 4U;
        chctl0 &= ~(TIMER_CHCTL0_CH1MS | TIMER_CHCTL0_CH1CAPFLT);
        chctl0 |= TIMER_IC_SELECTION_DIRECTTI << 8U | (extfilter << 12U);
    }else{
        chctl2 &= ~(TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP);
        chctl2 |= (uint32_t)extpolarity;
        chctl0 &= ~(TIMER_CHCTL0_CH0MS | TIMER_CHCTL0_CH0CAPFLT);
        chctl0 |= TIMER_IC_SELECTION_DIRECTTI | (extfilter << 4U);
    }

    TIMER_CHCTL0(timer_periph) = chctl0;
    TIMER_CHCTL2(timer_periph) = chctl2;
    timer_input_trigger_source_select(timer_periph,extrigger);
    TIMER_SMCFG(timer_periph) = (TIMER_SMCFG(timer_periph) & ~TIMER_SMCFG_SMC) | TIMER_SLAVE_MODE_EXTERNAL0;
}

/*!
    \brief      configure TIMER the external clock mode0
    \param[in]  timer_periph: TIMERx(x=0..4,7,8,11)
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
    \param[out] none
    \retval     none
*/
void timer_external_clock_mode0_config(uint32_t timer_periph, uint32_t extprescaler,
                                       uint32_t extpolarity, uint32_t extfilter)
{
    TIMER_SMCFG(timer_periph) = (TIMER_SMCFG(timer_periph) & ~(TIMER_SMCFG_SMC | TIMER_SMCFG_TRGS)) |
                                (TIMER_SLAVE_MODE_EXTERNAL0 | TIMER_SMCFG_TRGSEL_ETIFP);
    timer_external_trigger_config(timer_periph, extprescaler, extpolarity, extfilter);
}

/*!
    \brief      configure TIMER the external clock mode1
    \param[in]  timer_periph: TIMERx(x=0..4,7)
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
    \param[out] none
    \retval     none
*/
void timer_external_clock_mode1_config(uint32_t timer_periph, uint32_t extprescaler, uint32_t extpolarity, uint32_t extfilter)
{
    uint32_t reg = TIMER_SMCFG(timer_periph);
    reg &= ~TIMER_SMCFG_SMC1;
    reg |= (extprescaler | extpolarity | extfilter) << 12U;
    TIMER_SMCFG(timer_periph) = reg | TIMER_SMCFG_SMC1;
}

/*!
    \brief      disable TIMER the external clock mode1
    \param[in]  timer_periph: TIMERx(x=0..4,7)
    \param[out] none
    \retval     none
*/
void timer_external_clock_mode1_disable(uint32_t timer_periph)
{
    TIMER_SMCFG(timer_periph) &= ~(uint32_t)TIMER_SMCFG_SMC1;
}

/*!
    \brief      configure TIMER write CHxVAL register selection
    \param[in]  timer_periph: TIMERx(x=0..4,7..13)
    \param[in]  ccsel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CHVSEL_DISABLE: no effect
      \arg        TIMER_CHVSEL_ENABLE: when write the CHxVAL register, if the write value is same as the CHxVAL value, the write access is ignored
    \param[out] none
    \retval     none
*/
void timer_write_chxval_register_config(uint32_t timer_periph, uint16_t ccsel)
{
    TIMER_CFG(timer_periph) ^= (uint32_t)(ccsel == TIMER_CHVSEL_ENABLE) << TIMER_CFG_CHVSEL;
}

/*!
    \brief      configure TIMER output value selection
    \param[in]  timer_periph: TIMERx(x=0,7)
    \param[in]  outsel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_OUTSEL_DISABLE: no effect
      \arg        TIMER_OUTSEL_ENABLE: if POEN and IOS is 0, the output disabled
    \param[out] none
    \retval     none
*/
void timer_output_value_selection_config(uint32_t timer_periph, uint16_t outsel)
{
    if(TIMER_OUTSEL_ENABLE == outsel){
        TIMER_CFG(timer_periph) |= (uint32_t)TIMER_CFG_OUTSEL;
    }else if(TIMER_OUTSEL_DISABLE == outsel){
        TIMER_CFG(timer_periph) &= ~(uint32_t)TIMER_CFG_OUTSEL;
    }else{
        /* illegal parameters */
    }
}
