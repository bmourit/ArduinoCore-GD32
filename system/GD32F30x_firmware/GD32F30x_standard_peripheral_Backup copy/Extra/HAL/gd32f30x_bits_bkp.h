/**
 *  Copyright (c) 2024, BMourit ProjectHALGD
 *
 *  This HAL library is a distict, but derived from,
 *  the GigaDevice provided firmware files, so:
 *
 *  Copyright (c) 2020, GigaDevice Semiconductor Inc.
 *  
 *  Which in turn are clearly based on code from STMicroelectronics. We'll attempt
 *  to right the wrong of GigaDevice with:
 *
 *  Copyright (c) 2017, STMicroelectronics. All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without modification, 
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this 
 *      list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, 
 *      this list of conditions and the following disclaimer in the documentation 
 *      and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the names of its contributors 
 *      may be used to endorse or promote products derived from this software without 
 *      specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 *  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 *  OF SUCH DAMAGE.
 */

#ifndef GD32F30X_BITS_BKP_H
#define GD32F30X_BITS_BKP_H

#ifdef __cplusplus
extern "C" {
#endif

/* bits definitions */
/* BKP_DATA */
#define BKP_DATA                      BITS(0,15)               /*!< backup data */

/* BKP_OCTL */
#define BKP_OCTL_RCCV                 BITS(0,6)                /*!< RTC clock calibration value */
#define BKP_OCTL_COEN                 BIT(7)                   /*!< RTC clock calibration output enable */
#define BKP_OCTL_ASOEN                BIT(8)                   /*!< RTC alarm or second signal output enable */
#define BKP_OCTL_ROSEL                BIT(9)                   /*!< RTC output selection */
#define BKP_OCTL_CCOSEL               BIT(14)                  /*!< RTC clock output selection */
#define BKP_OCTL_CALDIR               BIT(15)                  /*!< RTC clock calibration direction */

/* BKP_TPCTL */
#define BKP_TPCTL_TPEN                BIT(0)                   /*!< tamper detection enable */
#define BKP_TPCTL_TPAL                BIT(1)                   /*!< tamper pin active level */

/* BKP_TPCS */
#define BKP_TPCS_TER                 BIT(0)                    /*!< tamper event reset */
#define BKP_TPCS_TIR                 BIT(1)                    /*!< tamper interrupt reset */
#define BKP_TPCS_TPIE                BIT(2)                    /*!< tamper interrupt enable */
#define BKP_TPCS_TEF                 BIT(8)                    /*!< tamper event flag */
#define BKP_TPCS_TIF                 BIT(9)                    /*!< tamper interrupt flag */

/* constants definitions */
/* BKP register */
#define BKP_DATA0_9(number)           REG16((BKP) + 0x04U + (number) * 0x04U)
#define BKP_DATA10_41(number)         REG16((BKP) + 0x40U + ((number)-10U) * 0x04U)

/* get data of BKP data register */
#define BKP_DATA_GET(regval)          GET_BITS((uint32_t)(regval), 0, 15)

/* RTC clock calibration value */
#define OCTL_RCCV(regval)             (BITS(0,6) & ((uint32_t)(regval) << 0))

/* RTC output selection */
#define RTC_OUTPUT_ALARM_PULSE        ((uint16_t)0x0000U)      /*!< RTC alarm pulse is selected as the RTC output */
#define RTC_OUTPUT_SECOND_PULSE       ((uint16_t)0x0200U)      /*!< RTC second pulse is selected as the RTC output */

/* RTC clock output selection */
#define RTC_CLOCK_DIV_64              ((uint16_t)0x0000U)      /*!< RTC clock div 64 */
#define RTC_CLOCK_DIV_1               ((uint16_t)0x4000U)      /*!< RTC clock div 1 */

/* RTC clock calibration direction */
#define RTC_CLOCK_SLOWED_DOWN         ((uint16_t)0x0000U)      /*!< RTC clock slow down */
#define RTC_CLOCK_SPEED_UP            ((uint16_t)0x8000U)      /*!< RTC clock speed up */

/* tamper pin active level */
#define TAMPER_PIN_ACTIVE_HIGH        ((uint16_t)0x0000U)      /*!< the tamper pin is active high */
#define TAMPER_PIN_ACTIVE_LOW         ((uint16_t)0x0002U)      /*!< the tamper pin is active low */

/* tamper flag */
#define BKP_FLAG_TAMPER                 BKP_TPCS_TEF             /*!< tamper event flag */

/* tamper interrupt flag */
#define BKP_INT_FLAG_TAMPER             BKP_TPCS_TIF             /*!< tamper interrupt flag */

/* BKP data register number */
typedef enum {
    BKP_DATA_0 = 1,
    BKP_DATA_1,
    BKP_DATA_2,
    BKP_DATA_3,
    BKP_DATA_4,
    BKP_DATA_5,
    BKP_DATA_6,
    BKP_DATA_7,
    BKP_DATA_8,
    BKP_DATA_9,
    BKP_DATA_10,
    BKP_DATA_11,
    BKP_DATA_12,
    BKP_DATA_13,
    BKP_DATA_14,
    BKP_DATA_15,
    BKP_DATA_16,
    BKP_DATA_17,
    BKP_DATA_18,
    BKP_DATA_19,
    BKP_DATA_20,
    BKP_DATA_21,
    BKP_DATA_22,
    BKP_DATA_23,
    BKP_DATA_24,
    BKP_DATA_25,
    BKP_DATA_26,
    BKP_DATA_27,
    BKP_DATA_28,
    BKP_DATA_29,
    BKP_DATA_30,
    BKP_DATA_31,
    BKP_DATA_32,
    BKP_DATA_33,
    BKP_DATA_34,
    BKP_DATA_35,
    BKP_DATA_36,
    BKP_DATA_37,
    BKP_DATA_38,
    BKP_DATA_39,
    BKP_DATA_40,
    BKP_DATA_41,
} bkp_data_register_enum;

#ifdef __cplusplus
}
#endif

#endif /* GD32F30X_BITS_BKP_H */
