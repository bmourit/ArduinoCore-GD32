/**
  ******************************************************************************
  * @file    GD32_eeprom.h
  * @brief   Header for eeprom module
  ******************************************************************************
  * @attention
  * 
  * Modified from STM32 for GD32
  * 
  * Copyright (c) 2016-2021, STMicroelectronics
  * All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_EEPROM_H
#define __DRV_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "gd32_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#ifndef FLASH_FLAG_ALL_ERRORS
#if defined(FLASH_FLAG_PGERR)
#if defined(FLASH_FLAG_OPTVERR)
#define FLASH_FLAG_ALL_ERRORS     (FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR | FLASH_FLAG_OPTVERR)
#else
#define FLASH_FLAG_ALL_ERRORS     (FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR)
#endif /* FLASH_FLAG_OPTVERR */
#else
#if defined(FLASH_PCROP_SUPPORT)
#define FLASH_FLAG_ALL_ERRORS     (FLASH_FLAG_OPERR   | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | \
                                   FLASH_FLAG_PGAERR  | FLASH_FLAG_SIZERR  | FLASH_FLAG_PGSERR | \
                                   FLASH_FLAG_MISERR  | FLASH_FLAG_FASTERR | FLASH_FLAG_RDERR  | \
                                   FLASH_FLAG_OPTVERR | FLASH_FLAG_ECCC    | FLASH_FLAG_ECCD)
#else
#define FLASH_FLAG_ALL_ERRORS     (FLASH_FLAG_OPERR   | FLASH_FLAG_PROGERR | FLASH_FLAG_WRPERR | \
                                   FLASH_FLAG_PGAERR  | FLASH_FLAG_SIZERR  | FLASH_FLAG_PGSERR | \
                                   FLASH_FLAG_MISERR  | FLASH_FLAG_FASTERR | \
                                   FLASH_FLAG_OPTVERR | FLASH_FLAG_ECCC    | FLASH_FLAG_ECCD)
#endif /* FLASH_PCROP_SUPPORT */
#endif /* FLASH_FLAG_PGERR */
#endif /* FLASH_FLAG_ALL_ERRORS */

#ifndef FLASH_PAGE_SIZE
/*
 * FLASH_PAGE_SIZE is not defined for STM32F2xx, STM32F4xx and STM32F7xx
 * Could be redefined in variant.h or using build_opt.h
 * Warning: This is not the sector size, only the size used for EEPROM
 * emulation. Anyway, all the sector size will be erased.
 * So pay attention to not use this sector for other stuff.
 */
#define FLASH_PAGE_SIZE     ((uint32_t)(8*1024)) /* 8kB page */
#endif

#if defined(DATA_EEPROM_BASE) || defined(FLASH_EEPROM_BASE)

#if defined (DATA_EEPROM_END)
#define E2END (DATA_EEPROM_END - DATA_EEPROM_BASE)
#elif defined (DATA_EEPROM_BANK2_END)
/* assuming two contiguous banks */
#define DATA_EEPROM_END DATA_EEPROM_BANK2_END
#define E2END (DATA_EEPROM_BANK2_END - DATA_EEPROM_BASE)
#elif defined (FLASH_EEPROM_END)
#define DATA_EEPROM_BASE FLASH_EEPROM_BASE
#define DATA_EEPROM_END FLASH_EEPROM_END
#define E2END (DATA_EEPROM_END - DATA_EEPROM_BASE)
#endif /* __EEPROM_END */

#else /* _EEPROM_BASE */
#define E2END (FLASH_PAGE_SIZE - 1)
#endif /* _EEPROM_BASE */

#endif /* STM32MP1xx */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

uint8_t eeprom_read_byte(const uint32_t pos);
void eeprom_write_byte(uint32_t pos, uint8_t value);

#if !defined(DATA_EEPROM_BASE)
void eeprom_buffer_fill();
void eeprom_buffer_flush();
uint8_t eeprom_buffered_read_byte(const uint32_t pos);
void eeprom_buffered_write_byte(uint32_t pos, uint8_t value);
#endif /* ! DATA_EEPROM_BASE */

#ifdef __cplusplus
}
#endif

#endif /* __DRV_EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
