/*!
    \file    gd32f3x0_libopt.h
    \brief   library optional for gd32f3x0

    \version 2017-06-06, V1.0.0, firmware for GD32F3x0
    \version 2019-06-01, V2.0.0, firmware for GD32F3x0
    \version 2020-09-30, V2.1.0, firmware for GD32F3x0
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

#ifndef GD32F3X0_LIBOPT_H
#define GD32F3X0_LIBOPT_H

#include "gd32xxyy_spl_config.h"

#if defined(SPL_ADC_ENABLE)
#include "gd32f3x0_adc.h"
#endif
#if defined(SPL_CRC_ENABLE)
#include "gd32f3x0_crc.h"
#endif
#if defined(SPL_CTC_ENABLE)
#include "gd32f3x0_ctc.h"
#endif
#if defined(SPL_DBG_ENABLE)
#include "gd32f3x0_dbg.h"
#endif
#if defined(SPL_DMA_ENABLE)
#include "gd32f3x0_dma.h"
#endif
#if defined(SPL_EXTI_ENABLE)
#include "gd32f3x0_exti.h"
#endif
#if defined(SPL_FMC_ENABLE)
#include "gd32f3x0_fmc.h"
#endif
#if defined(SPL_GPIO_ENABLE)
#include "gd32f3x0_gpio.h"
#endif
#if defined(SPL_SYSCFG_ENABLE)
#include "gd32f3x0_syscfg.h"
#endif
#if defined(SPL_I2C_ENABLE)
#include "gd32f3x0_i2c.h"
#endif
#if defined(SPL_FWDGT_ENABLE)
#include "gd32f3x0_fwdgt.h"
#endif
#if defined(SPL_PMU_ENABLE)
#include "gd32f3x0_pmu.h"
#endif
#if defined(SPL_RCU_ENABLE)
#include "gd32f3x0_rcu.h"
#endif
#if defined(SPL_RTC_ENABLE)
#include "gd32f3x0_rtc.h"
#endif
#if defined(SPL_SPI_ENABLE)
#include "gd32f3x0_spi.h"
#endif
#if defined(SPL_TIMER_ENABLE)
#include "gd32f3x0_timer.h"
#endif
#if defined(SPL_USART_ENABLE)
#include "gd32f3x0_usart.h"
#endif
#if defined(SPL_WWDGT_ENABLE)
#include "gd32f3x0_wwdgt.h"
#endif
#if defined(SPL_MISC_ENABLE)
#include "gd32f3x0_misc.h"
#endif
#if defined(SPL_TSI_ENABLE)
#include "gd32f3x0_tsi.h"
#endif
#if defined(SPL_CEC_ENABLE)
#include "gd32f3x0_cec.h"
#endif
#if defined(SPL_CMP_ENABLE)
#include "gd32f3x0_cmp.h"
#endif
#if defined(SPL_DAC_ENABLE)
#include "gd32f3x0_dac.h"
#endif

#endif /* GD32F3X0_LIBOPT_H */
