/*!
    \file    gd32f30x_libopt.h
    \brief   library optional for gd32f30x

    \version 2021-03-23, V2.0.0, demo for GD32F30x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

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

#ifndef GD32F30X_LIBOPT_H
#define GD32F30X_LIBOPT_H

#include "gd32xxyy_spl_config.h"

#if defined(SPL_RCU_ENABLE)
#include "gd32f30x_rcu.h"
#endif
#if defined(SPL_ADC_ENABLE)
#include "gd32f30x_adc.h"
#endif
#if defined(SPL_CAN_ENABLE)
#include "gd32f30x_can.h"
#endif
#if defined(SPL_CRC_ENABLE)
#include "gd32f30x_crc.h"
#endif
#if defined(SPL_CTC_ENABLE)
#include "gd32f30x_ctc.h"
#endif
#if defined(SPL_DAC_ENABLE)
#include "gd32f30x_dac.h"
#endif
#if defined(SPL_DBG_ENABLE)
#include "gd32f30x_dbg.h"
#endif
#if defined(SPL_DMA_ENABLE)
#include "gd32f30x_dma.h"
#endif
#if defined(SPL_EXTI_ENABLE)
#include "gd32f30x_exti.h"
#endif
#if defined(SPL_FMC_ENABLE)
#include "gd32f30x_fmc.h"
#endif
#if defined(SPL_FWDGT_ENABLE)
#include "gd32f30x_fwdgt.h"
#endif
#if defined(SPL_GPIO_ENABLE)
#include "gd32f30x_gpio.h"
#endif
#if defined(SPL_I2C_ENABLE)
#include "gd32f30x_i2c.h"
#endif
#if defined(SPL_PMU_ENABLE)
#include "gd32f30x_pmu.h"
#endif
#if defined(SPL_BKP_ENABLE)
#include "gd32f30x_bkp.h"
#endif
#if defined(SPL_RTC_ENABLE)
#include "gd32f30x_rtc.h"
#endif
#if defined(SPL_SDIO_ENABLE)
#include "gd32f30x_sdio.h"
#endif
#if defined(SPL_SPI_ENABLE)
#include "gd32f30x_spi.h"
#endif
#if defined(SPL_TIMER_ENABLE)
#include "gd32f30x_timer.h"
#endif
#if defined(SPL_USART_ENABLE)
#include "gd32f30x_usart.h"
#endif
#if defined(SPL_WWDGT_ENABLE)
#include "gd32f30x_wwdgt.h"
#endif
#if defined(SPL_MISC_ENABLE)
#include "gd32f30x_misc.h"
#endif
#if defined(SPL_ENET_ENABLE)
#include "gd32f30x_enet.h"
#endif
#if defined(SPL_EXMC_ENABLE)
#include "gd32f30x_exmc.h"
#endif

#endif /* GD32F30X_LIBOPT_H */
