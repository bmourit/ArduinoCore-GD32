#ifndef _GD32XXYY_SPL_CONFIG_H_
#define _GD32XXYY_SPL_CONFIG_H_

/**
 * modules are set to be included by default
 * to exclude, define SPL_X_DISABLE
 * (X = peripheral) in board variant file
 */
#include "variant.h"

#define SPL_FMC_ENABLE
#define SPL_MISC_ENABLE
#define SPL_DMA_ENABLE
#define SPL_GPIO_ENABLE
#define SPL_PMU_ENABLE
#define SPL_RCU_ENABLE
#define SPL_USART_ENABLE

#if !defined(SAFE_CLOCKS_DISABLE)
   #define SAFE_CLOCKS_ENABLE
#else
   #undef SAFE_CLOCKS_ENABLE
#endif

#if !defined(SPL_ADC_DISABLE)
   #define SPL_ADC_ENABLE
#else
   #undef SPL_ADC_ENABLE
#endif

#if !defined(SPL_I2C_DISABLE)
   #define SPL_I2C_ENABLE
#else
   #undef SPL_I2C_ENABLE
#endif

#if !defined(SPL_RTC_DISABLE)
   #define SPL_RTC_ENABLE
#else
   #undef SPL_RTC_ENABLE
#endif

#if !defined(SPL_SPI_DISABLE)
   #define SPL_SPI_ENABLE
#else
   #undef SPL_SPI_ENABLE
#endif

#if !defined(SPL_TIMER_DISABLE)
   #define SPL_TIMER_ENABLE
#else
   #undef SPL_TIMER_ENABLE
#endif

#if !defined(SPL_DAC_DISABLE)
   #define SPL_DAC_ENABLE
#else
   #undef SPL_DAC_ENABLE
#endif

#if !defined(SPL_EXTI_DISABLE)
   #define SPL_EXTI_ENABLE
#else
   #undef SPL_EXTI_ENABLE
#endif

#if !defined(SPL_ENET_DISABLE)
   #define SPL_ENET_ENABLE
#else
   #undef SPL_ENET_ENABLE
#endif

#if !defined(SPL_SDIO_DISABLE)
   #define SPL_SDIO_ENABLE
#else
   #undef SPL_SDIO_ENABLE
#endif

#if !defined(SPL_FWDGT_DISABLE)
   #define SPL_FWDGT_ENABLE
#else
   #undef SPL_FWDGT_ENABLE
#endif

#if !defined(SPL_CAN_DISABLE)
   #define SPL_CAN_ENABLE
#else
   #undef SPL_CAN_ENABLE
#endif

#if !defined(SPL_CRC_DISABLE)
   #define SPL_CRC_ENABLE
#else
   #undef SPL_CRC_ENABLE
#endif

#if !defined(SPL_CTC_DISABLE)
   #define SPL_CTC_ENABLE
#else
   #undef SPL_CTC_ENABLE
#endif

#if !defined(SPL_BKP_DISABLE)
   #define SPL_BKP_ENABLE
#else
   #undef SPL_BKP_ENABLE
#endif

#if !defined(SPL_WWDGT_DISABLE)
   #define SPL_WWDGT_ENABLE
#else
   #undef SPL_WWDGT_ENABLE
#endif

#if !defined(SPL_DBG_DISABLE)
   #define SPL_DBG_ENABLE
#else
   #undef SPL_DBG_ENABLE
#endif

#if !defined(SPL_EXMC_DISABLE)
   #define SPL_EXMC_ENABLE
#else
   #undef SPL_EXMC_ENABLE
#endif

//#include "gd32f30x_rcu.h"
///#include "gd32f30x_adc.h"
//#include "gd32f30x_can.h"
//#include "gd32f30x_crc.h"
//#include "gd32f30x_ctc.h"
//#include "gd32f30x_dac.h"
//#include "gd32f30x_dbg.h"
//#include "gd32f30x_dma.h"
//#include "gd32f30x_exti.h"
//#include "gd32f30x_fmc.h"
//#include "gd32f30x_fwdgt.h"
//#include "gd32f30x_gpio.h"
//#include "gd32f30x_i2c.h"
//#include "gd32f30x_pmu.h"
//#include "gd32f30x_bkp.h"
//#include "gd32f30x_rtc.h"
//#include "gd32f30x_sdio.h"
//#include "gd32f30x_spi.h"
//#include "gd32f30x_timer.h"
//#include "gd32f30x_usart.h"
//#include "gd32f30x_wwdgt.h"
//#include "gd32f30x_misc.h"
//#include "gd32f30x_enet.h"
//#include "gd32f30x_exmc.h"

#endif /* _GD32XXYY_SPL_CONFIG_H_ */
